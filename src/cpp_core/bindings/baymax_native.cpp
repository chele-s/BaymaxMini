#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "serial/SerialLink.h"
#include "analysis/AdvancedVitalsAnalyzer.h"
#include "ipc/SharedData.h"

namespace py = pybind11;

PYBIND11_MODULE(baymax_native, m) {
    m.doc() = "BaymaxMini Native C++ Module";

    py::enum_<ipc::ExpressionType>(m, "ExpressionType")
        .value("NEUTRAL", ipc::ExpressionType::NEUTRAL)
        .value("HAPPY", ipc::ExpressionType::HAPPY)
        .value("SAD", ipc::ExpressionType::SAD)
        .value("SURPRISED", ipc::ExpressionType::SURPRISED)
        .value("ANGRY", ipc::ExpressionType::ANGRY)
        .value("SLEEPY", ipc::ExpressionType::SLEEPY)
        .value("CONCERNED", ipc::ExpressionType::CONCERNED)
        .value("CURIOUS", ipc::ExpressionType::CURIOUS)
        .value("LOVE", ipc::ExpressionType::LOVE)
        .value("THINKING", ipc::ExpressionType::THINKING)
        .export_values();

    py::enum_<ipc::SystemStateId>(m, "SystemStateId")
        .value("BOOT", ipc::SystemStateId::BOOT)
        .value("CONNECTED", ipc::SystemStateId::CONNECTED)
        .value("AUTONOMOUS", ipc::SystemStateId::AUTONOMOUS)
        .value("SHUTDOWN", ipc::SystemStateId::SHUTDOWN)
        .value("ERROR", ipc::SystemStateId::ERROR)
        .export_values();

    py::enum_<ipc::AlertLevel>(m, "AlertLevel")
        .value("NONE", ipc::AlertLevel::NONE)
        .value("INFO", ipc::AlertLevel::INFO)
        .value("WARNING", ipc::AlertLevel::WARNING)
        .value("CRITICAL", ipc::AlertLevel::CRITICAL)
        .export_values();

    py::class_<ipc::TelemetryFrame>(m, "TelemetryFrame")
        .def(py::init<>())
        .def_readwrite("sequence", &ipc::TelemetryFrame::sequence)
        .def_readwrite("timestamp_us", &ipc::TelemetryFrame::timestamp_us)
        .def_readwrite("distance_mm", &ipc::TelemetryFrame::distance_mm)
        .def_readwrite("heart_rate_bpm", &ipc::TelemetryFrame::heart_rate_bpm)
        .def_readwrite("spo2_percent", &ipc::TelemetryFrame::spo2_percent)
        .def_readwrite("skin_temp_c", &ipc::TelemetryFrame::skin_temp_c)
        .def_readwrite("ambient_temp_c", &ipc::TelemetryFrame::ambient_temp_c)
        .def_readwrite("bus_voltage_v", &ipc::TelemetryFrame::bus_voltage_v)
        .def_readwrite("current_ma", &ipc::TelemetryFrame::current_ma)
        .def_readwrite("power_mw", &ipc::TelemetryFrame::power_mw)
        .def_readwrite("battery_pct", &ipc::TelemetryFrame::battery_pct)
        .def_readwrite("eyelid_openness", &ipc::TelemetryFrame::eyelid_openness)
        .def_readwrite("gaze_x", &ipc::TelemetryFrame::gaze_x)
        .def_readwrite("gaze_y", &ipc::TelemetryFrame::gaze_y)
        .def_readwrite("breath_level", &ipc::TelemetryFrame::breath_level)
        .def_readwrite("state", &ipc::TelemetryFrame::state)
        .def_readwrite("expression", &ipc::TelemetryFrame::expression)
        .def_readwrite("alert", &ipc::TelemetryFrame::alert);

    py::class_<SerialLink::Stats>(m, "SerialStats")
        .def_readwrite("bytesReceived", &SerialLink::Stats::bytesReceived)
        .def_readwrite("bytesSent", &SerialLink::Stats::bytesSent)
        .def_readwrite("packetsRx", &SerialLink::Stats::packetsRx)
        .def_readwrite("packetsTx", &SerialLink::Stats::packetsTx)
        .def_readwrite("checksumFails", &SerialLink::Stats::checksumFails)
        .def_readwrite("readErrors", &SerialLink::Stats::readErrors);

    py::class_<SerialLink>(m, "SerialLink")
        .def(py::init<>())
        .def("open_port", &SerialLink::openPort, py::arg("port_name"), py::arg("baud_rate") = 921600)
        .def("close_port", &SerialLink::closePort)
        .def("is_open", &SerialLink::isOpen)
        .def("get_latest_telemetry", [](SerialLink& self) -> py::object {
            ipc::TelemetryFrame frame;
            if (self.getLatestTelemetry(frame)) {
                return py::cast(frame);
            }
            return py::none();
        })
        .def("send_expression", &SerialLink::sendExpression, py::arg("expr"), py::arg("transition_sec") = 0.3f)
        .def("send_eyelid", &SerialLink::sendEyelid, py::arg("openness"), py::arg("duration_sec") = 0.2f)
        .def("send_gaze", &SerialLink::sendGaze, py::arg("x"), py::arg("y"), py::arg("speed") = 1.0f)
        .def("send_breath", &SerialLink::sendBreath, py::arg("intensity"))
        .def("send_emergency_stop", &SerialLink::sendEmergencyStop)
        .def("send_shutdown", &SerialLink::sendShutdown)
        .def("get_stats", &SerialLink::getStats)
        .def("reset_stats", &SerialLink::resetStats);

    py::class_<HRVAnalysisResult>(m, "HRVAnalysisResult")
        .def_readwrite("meanRRMs", &HRVAnalysisResult::meanRRMs)
        .def_readwrite("sdnnMs", &HRVAnalysisResult::sdnnMs)
        .def_readwrite("rmssdMs", &HRVAnalysisResult::rmssdMs)
        .def_readwrite("pnn50Pct", &HRVAnalysisResult::pnn50Pct)
        .def_readwrite("lfPower", &HRVAnalysisResult::lfPower)
        .def_readwrite("hfPower", &HRVAnalysisResult::hfPower)
        .def_readwrite("lfHfRatio", &HRVAnalysisResult::lfHfRatio)
        .def_readwrite("stressIndex", &HRVAnalysisResult::stressIndex)
        .def_readwrite("valid", &HRVAnalysisResult::valid);

    py::class_<VitalsRiskAssessment>(m, "VitalsRiskAssessment")
        .def_readwrite("overallRiskScore", &VitalsRiskAssessment::overallRiskScore)
        .def_readwrite("cardiacRisk", &VitalsRiskAssessment::cardiacRisk)
        .def_readwrite("respiratoryRisk", &VitalsRiskAssessment::respiratoryRisk)
        .def_readwrite("thermalRisk", &VitalsRiskAssessment::thermalRisk)
        .def_readwrite("vitalAlertTriggered", &VitalsRiskAssessment::vitalAlertTriggered);

    py::class_<AdvancedVitalsAnalyzer>(m, "AdvancedVitalsAnalyzer")
        .def(py::init<>())
        .def("add_heart_rate_sample", &AdvancedVitalsAnalyzer::addHeartRateSample, py::arg("bpm"), py::arg("timestamp_sec"))
        .def("add_spo2_sample", &AdvancedVitalsAnalyzer::addSpO2Sample, py::arg("spo2"), py::arg("timestamp_sec"))
        .def("add_temperature_sample", &AdvancedVitalsAnalyzer::addTemperatureSample, py::arg("temp_c"), py::arg("timestamp_sec"))
        .def("analyze_hrv", &AdvancedVitalsAnalyzer::analyzeHRV)
        .def("evaluate_health_risk", &AdvancedVitalsAnalyzer::evaluateHealthRisk, py::arg("hr"), py::arg("spo2"), py::arg("temp_c"))
        .def("reset_history", &AdvancedVitalsAnalyzer::resetHistory);
}
