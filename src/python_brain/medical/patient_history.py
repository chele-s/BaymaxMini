import sqlite3
import time
from typing import Any, Dict, List, Optional
from datetime import datetime

from ..core.event_bus import EventBus
from ..utils.logger import get_logger
from config.settings import CONFIG

class PatientHistory:
    def __init__(self, event_bus: EventBus) -> None:
        self.bus = event_bus
        self.logger = get_logger()
        self.db_path = CONFIG.get("medical.db_path", "config/medicines.db")
        self._running = False
        self._setup_tables()

    def _setup_tables(self) -> None:
        try:
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    CREATE TABLE IF NOT EXISTS vitals_log (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp REAL NOT NULL,
                        heart_rate_bpm REAL,
                        spo2_percent REAL,
                        skin_temp_c REAL,
                        ambient_temp_c REAL,
                        pain_estimate INTEGER
                    )
                """)
                conn.commit()
        except sqlite3.Error as e:
            self.logger.error(f"Failed to setup patient history tables: {e}")

    def start(self) -> bool:
        if self._running:
            return True
        self.bus.subscribe("TELEMETRY_UPDATE", self._on_telemetry)
        self.bus.subscribe("EVALUATE_TRENDS", self._on_evaluate_trends)
        self._running = True
        return True

    def stop(self) -> None:
        self._running = False
        self.bus.unsubscribe("TELEMETRY_UPDATE", self._on_telemetry)
        self.bus.unsubscribe("EVALUATE_TRENDS", self._on_evaluate_trends)

    def _on_telemetry(self, data: Any) -> None:
        if not self._running or not isinstance(data, dict):
            return
            
        if not data.get("vitals_valid", False):
            return
            
        current_time = time.time()
        hr = data.get("heart_rate_bpm")
        spo2 = data.get("spo2_percent")
        skin_temp = data.get("skin_temp_c")
        amb_temp = data.get("ambient_temp_c")
        pain = data.get("alert", 0) 
        
        try:
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT INTO vitals_log (timestamp, heart_rate_bpm, spo2_percent, skin_temp_c, ambient_temp_c, pain_estimate)
                    VALUES (?, ?, ?, ?, ?, ?)
                """, (current_time, hr, spo2, skin_temp, amb_temp, pain))
                conn.commit()
        except sqlite3.Error as e:
            self.logger.error(f"DB Insert failed: {e}")

    def get_recent_metrics(self, hours: float = 24.0) -> List[Dict[str, Any]]:
        threshold = time.time() - (hours * 3600.0)
        results = []
        try:
            with sqlite3.connect(self.db_path) as conn:
                conn.row_factory = sqlite3.Row
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT timestamp, heart_rate_bpm, spo2_percent, skin_temp_c, ambient_temp_c, pain_estimate
                    FROM vitals_log
                    WHERE timestamp >= ?
                    ORDER BY timestamp ASC
                """, (threshold,))
                
                for row in cursor.fetchall():
                    results.append(dict(row))
        except sqlite3.Error as e:
            self.logger.error(f"DB Select failed: {e}")
            
        return results

    def _on_evaluate_trends(self, data: Any) -> None:
        metrics = self.get_recent_metrics(hours=72.0)
        if not metrics:
            return
            
        temps = [m["skin_temp_c"] for m in metrics if m["skin_temp_c"] is not None]
        if not temps:
            return
            
        avg_temp = sum(temps) / len(temps)
        latest_temp = temps[-1]
        
        if latest_temp > avg_temp + 1.5:
            self.bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "concerned", "transition": 1.0}})
            self.bus.publish("COMMAND_SPEAK", {"text": "I noticed your temperature is unusually high compared to the last three days. You should rest."})
        elif latest_temp > 37.8:
            self.bus.publish("HIGH_TEMPERATURE", {"temperature": latest_temp, "timestamp": time.time()})
