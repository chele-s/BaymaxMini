from typing import Optional
from pydantic import BaseModel, Field, field_validator

class Telemetry(BaseModel):
    sequence: int = Field(..., description="Chronological packet sequence number")
    timestamp_us: int = Field(..., description="Microsecond timestamp from C++ core")
    
    distance_mm: float = Field(..., ge=0, description="ToF sensor distance in mm")
    heart_rate_bpm: float = Field(..., ge=0, le=250, description="MAX30102 Heart rate")
    spo2_percent: float = Field(..., ge=0, le=100, description="MAX30102 SpO2 percentage")
    
    skin_temp_c: float = Field(..., ge=-20, le=60, description="Patient skin temperature Celsius")
    ambient_temp_c: float = Field(..., ge=-20, le=60, description="Environment temperature Celsius")
    
    bus_voltage_v: float = Field(..., ge=0, description="INA219 bus voltage")
    current_ma: float = Field(..., description="INA219 current draw in mA")
    power_mw: float = Field(..., description="INA219 power computation in mW")
    
    proximity_valid: bool = Field(..., description="Flag indicating distance_mm is reliable")
    vitals_valid: bool = Field(..., description="Flag indicating heart_rate and spo2 are reliable")
    power_valid: bool = Field(..., description="Flag indicating INA219 readings are reliable")
    face_valid: bool = Field(..., description="Flag indicating face tracking data is reliable")
    
    battery_pct: int = Field(..., ge=0, le=100, description="Estimated battery percentage remaining")
    
    eyelid_openness: float = Field(..., ge=0.0, le=1.0, description="Eyelid state from face tracking")
    gaze_x: float = Field(..., ge=-1.0, le=1.0, description="Normalized horizontal gaze vector")
    gaze_y: float = Field(..., ge=-1.0, le=1.0, description="Normalized vertical gaze vector")
    
    expression: int = Field(..., description="Enum representation of current facial expression")
    state: int = Field(..., description="Enum representation of C++ core state")
    alert: int = Field(..., description="Enum representation of active hardware alerts")

    @field_validator('skin_temp_c', 'ambient_temp_c')
    def validate_temperatures(cls, v: float) -> float:
        if v > 50.0 or v < -10.0:
            raise ValueError(f"Temperature reading {v} is outside plausible hardware limits.")
        return v

    @field_validator('battery_pct')
    def validate_battery(cls, v: int) -> int:
        if v > 100 or v < 0:
            raise ValueError(f"Battery percentage {v} is invalid.")
        return v
