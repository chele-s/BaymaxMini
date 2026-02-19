from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Optional

import yaml
from pydantic import BaseModel, Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


_CONFIG_PATH = Path(__file__).parent / "settings.yaml"


class SystemSettings(BaseModel):
    name: str = "BaymaxMini"
    version: str = "1.0.0"
    log_level: str = "INFO"
    log_dir: str = "/var/log/baymax"
    log_max_bytes: int = 10_485_760
    log_backup_count: int = 5
    locale: str = "es_MX"


class PathSettings(BaseModel):
    db: Path = Path("config/medicines.db")
    yolo_model: Path = Path("config/neural_net/yolo_v8n.pt")
    face_landmark_model: Path = Path("config/neural_net/face_land.task")
    sounds_dir: Path = Path("assets/sounds")


class ZmqSettings(BaseModel):
    telemetry_endpoint: str = "tcp://127.0.0.1:5556"
    command_endpoint: str = "tcp://127.0.0.1:5555"
    topic_telem: str = "TELEM"
    topic_cmd: str = "CMD"
    sub_linger_ms: int = 0
    pub_hwm: int = 1
    heartbeat_interval_s: float = 1.0
    brain_timeout_s: float = 2.0


class VisionSettings(BaseModel):
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: int = 30
    camera_format: str = "RGB888"
    detector_confidence: float = Field(0.55, ge=0.0, le=1.0)
    detector_iou: float = Field(0.45, ge=0.0, le=1.0)
    detector_device: str = "cpu"
    detector_half: bool = False
    face_min_detection_confidence: float = Field(0.6, ge=0.0, le=1.0)
    face_min_tracking_confidence: float = Field(0.5, ge=0.0, le=1.0)
    visual_memory_ttl_s: float = 1.5
    frame_queue_maxsize: int = 2
    result_queue_maxsize: int = 8


class AudioSettings(BaseModel):
    stt_model_path: str = "/opt/vosk/model-es"
    stt_sample_rate: int = 16000
    stt_block_size: int = 8000
    tts_model_path: str = "/opt/piper/es_MX-claude-high.onnx"
    tts_sample_rate: int = 22050
    tts_speaker_id: int = 0
    sounds_volume: float = Field(0.8, ge=0.0, le=1.0)
    mic_device_index: Optional[int] = None


class MedicalSettings(BaseModel):
    fever_threshold_c: float = 37.5
    hypothermia_threshold_c: float = 35.0
    tachycardia_bpm: int = 100
    bradycardia_bpm: int = 50
    low_spo2_pct: float = 94.0
    reminder_lead_time_s: int = 30
    missed_dose_window_s: int = 300


class PowerSettings(BaseModel):
    nominal_voltage_v: float = 11.1
    full_voltage_v: float = 12.6
    empty_voltage_v: float = 9.0
    capacity_mah: float = 2200.0
    cell_count: int = 3
    overcurrent_trip_ma: float = 3000.0
    overcurrent_clear_ma: float = 2500.0
    servo_stall_trip_ma: float = 2000.0
    servo_stall_clear_ma: float = 1200.0
    low_battery_pct: float = 15.0
    critical_battery_pct: float = 5.0


class AppSettings(BaseSettings):
    model_config = SettingsConfigDict(
        env_prefix="BAYMAX_",
        env_nested_delimiter="__",
        case_sensitive=False,
        extra="ignore",
    )

    system: SystemSettings = SystemSettings()
    paths: PathSettings = PathSettings()
    zmq: ZmqSettings = ZmqSettings()
    vision: VisionSettings = VisionSettings()
    audio: AudioSettings = AudioSettings()
    medical: MedicalSettings = MedicalSettings()
    power: PowerSettings = PowerSettings()

    @field_validator("system", mode="before")
    @classmethod
    def _coerce_system(cls, v: object) -> object:
        return v

    @classmethod
    def from_yaml(cls, path: Path = _CONFIG_PATH) -> "AppSettings":
        raw: dict = {}
        if path.exists():
            with path.open("r", encoding="utf-8") as fh:
                raw = yaml.safe_load(fh) or {}
        return cls(**raw)


@lru_cache(maxsize=1)
def get_settings() -> AppSettings:
    return AppSettings.from_yaml()
