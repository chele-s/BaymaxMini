#pragma once

#include <cstdint>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include "Config.h"

class CamServer {
public:
    enum class Mode : uint8_t {
        DISABLED,
        AP,
        STA
    };

    struct StreamStats {
        uint32_t framesSent    = 0;
        uint32_t clientCount   = 0;
        float    avgFps        = 0.0f;
        uint32_t lastFrameMs   = 0;
    };

    CamServer() 
        : m_server(nullptr)
        , m_mode(Mode::DISABLED)
        , m_cameraReady(false)
    {}

    bool initCamera() {
        camera_config_t config;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer   = LEDC_TIMER_0;
        config.pin_d0       = cfg::CAM_PIN_D0;
        config.pin_d1       = cfg::CAM_PIN_D1;
        config.pin_d2       = cfg::CAM_PIN_D2;
        config.pin_d3       = cfg::CAM_PIN_D3;
        config.pin_d4       = cfg::CAM_PIN_D4;
        config.pin_d5       = cfg::CAM_PIN_D5;
        config.pin_d6       = cfg::CAM_PIN_D6;
        config.pin_d7       = cfg::CAM_PIN_D7;
        config.pin_xclk     = cfg::CAM_PIN_XCLK;
        config.pin_pclk     = cfg::CAM_PIN_PCLK;
        config.pin_vsync    = cfg::CAM_PIN_VSYNC;
        config.pin_href     = cfg::CAM_PIN_HREF;
        config.pin_sccb_sda = cfg::CAM_PIN_SIOD;
        config.pin_sccb_scl = cfg::CAM_PIN_SIOC;
        config.pin_pwdn     = cfg::CAM_PIN_PWDN;
        config.pin_reset    = cfg::CAM_PIN_RESET;
        config.xclk_freq_hz = 20000000;
        config.pixel_format = PIXFORMAT_JPEG;
        config.frame_size   = static_cast<framesize_t>(cfg::CAM_FRAME_SIZE);
        config.jpeg_quality = cfg::CAM_JPEG_QUALITY;
        config.fb_count     = cfg::CAM_FB_COUNT;
        config.fb_location  = CAMERA_FB_IN_PSRAM;
        config.grab_mode    = CAMERA_GRAB_LATEST;

        esp_err_t err = esp_camera_init(&config);
        if (err != ESP_OK) return false;

        m_cameraReady = true;
        return true;
    }

    bool startAP(const char* ssid, const char* password = nullptr) {
        if (!m_cameraReady) return false;

        if (password && strlen(password) > 0) {
            WiFi.softAP(ssid, password, cfg::WIFI_CHANNEL);
        } else {
            WiFi.softAP(ssid, nullptr, cfg::WIFI_CHANNEL);
        }

        m_mode = Mode::AP;
        return startHttpServer();
    }

    bool startSTA(const char* ssid, const char* password, uint32_t timeoutMs = 10000) {
        if (!m_cameraReady) return false;

        WiFi.begin(ssid, password);

        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - start > timeoutMs) return false;
            delay(100);
        }

        m_mode = Mode::STA;
        return startHttpServer();
    }

    void stop() {
        if (m_server) {
            httpd_stop(m_server);
            m_server = nullptr;
        }
        if (m_mode == Mode::AP) {
            WiFi.softAPdisconnect(true);
        } else if (m_mode == Mode::STA) {
            WiFi.disconnect(true);
        }
        m_mode = Mode::DISABLED;
    }

    bool isRunning() const {
        return m_server != nullptr && m_cameraReady;
    }

    IPAddress localIP() const {
        if (m_mode == Mode::AP) return WiFi.softAPIP();
        return WiFi.localIP();
    }

    const StreamStats& stats() const {
        return m_stats;
    }

    Mode mode() const {
        return m_mode;
    }

private:
    static esp_err_t streamHandler(httpd_req_t* req) {
        CamServer* self = static_cast<CamServer*>(req->user_ctx);

        httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

        static const char* BOUNDARY = "\r\n--frame\r\n";
        static const char* CONTENT_TYPE = "Content-Type: image/jpeg\r\nContent-Length: ";
        static const char* CRLF = "\r\n\r\n";

        char lenBuf[16];

        while (true) {
            camera_fb_t* fb = esp_camera_fb_get();
            if (!fb) {
                continue;
            }

            httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY));
            httpd_resp_send_chunk(req, CONTENT_TYPE, strlen(CONTENT_TYPE));

            snprintf(lenBuf, sizeof(lenBuf), "%u", fb->len);
            httpd_resp_send_chunk(req, lenBuf, strlen(lenBuf));
            httpd_resp_send_chunk(req, CRLF, strlen(CRLF));

            esp_err_t res = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(fb->buf), fb->len);

            esp_camera_fb_return(fb);

            ++self->m_stats.framesSent;

            uint32_t now = millis();
            if (self->m_stats.lastFrameMs > 0) {
                float dt = static_cast<float>(now - self->m_stats.lastFrameMs);
                if (dt > 0.0f) {
                    float instantFps = 1000.0f / dt;
                    self->m_stats.avgFps = self->m_stats.avgFps * 0.9f + instantFps * 0.1f;
                }
            }
            self->m_stats.lastFrameMs = now;

            if (res != ESP_OK) break;
        }

        return ESP_OK;
    }

    static esp_err_t indexHandler(httpd_req_t* req) {
        httpd_resp_set_type(req, "text/html");
        static const char HTML[] =
            "<!DOCTYPE html><html><head><title>BaymaxMini</title>"
            "<style>body{margin:0;background:#000;display:flex;"
            "justify-content:center;align-items:center;height:100vh}"
            "img{max-width:100%;max-height:100vh}</style></head>"
            "<body><img src=\"/stream\"></body></html>";
        httpd_resp_send(req, HTML, strlen(HTML));
        return ESP_OK;
    }

    bool startHttpServer() {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = cfg::CAM_MJPEG_PORT;
        config.ctrl_port   = cfg::CAM_MJPEG_PORT + 1;
        config.stack_size  = 8192;

        if (httpd_start(&m_server, &config) != ESP_OK) {
            return false;
        }

        httpd_uri_t streamUri;
        streamUri.uri     = "/stream";
        streamUri.method  = HTTP_GET;
        streamUri.handler = streamHandler;
        streamUri.user_ctx = this;
        httpd_register_uri_handler(m_server, &streamUri);

        httpd_uri_t indexUri;
        indexUri.uri     = "/";
        indexUri.method  = HTTP_GET;
        indexUri.handler = indexHandler;
        indexUri.user_ctx = this;
        httpd_register_uri_handler(m_server, &indexUri);

        return true;
    }

    httpd_handle_t  m_server;
    Mode            m_mode;
    bool            m_cameraReady;
    StreamStats     m_stats;
};
