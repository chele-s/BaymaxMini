import asyncio
import logging
import sqlite3
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from apscheduler.schedulers.asyncio import AsyncIOScheduler

from ..core.event_bus import EventBus

class Pharmacist:
    def __init__(self, event_bus: EventBus, db_path: str = "baymax_medical.db") -> None:
        self._bus = event_bus
        self._db_path = db_path
        self._scheduler = AsyncIOScheduler()
        self._logger = logging.getLogger(__name__)
        
        self._last_meal_time: float = 0.0
        self._last_water_time: float = 0.0
        
        self._setup_database()
        self._register_subscriptions()

    def _setup_database(self) -> None:
        try:
            with sqlite3.connect(self._db_path) as conn:
                cursor = conn.cursor()
                
                cursor.execute("""
                    CREATE TABLE IF NOT EXISTS medications (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        name TEXT UNIQUE NOT NULL,
                        dosage TEXT NOT NULL,
                        requires_food BOOLEAN NOT NULL DEFAULT 0,
                        requires_water BOOLEAN NOT NULL DEFAULT 1,
                        active BOOLEAN NOT NULL DEFAULT 1
                    )
                """)
                
                cursor.execute("""
                    CREATE TABLE IF NOT EXISTS schedules (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        medication_id INTEGER,
                        time_str TEXT NOT NULL,
                        FOREIGN KEY (medication_id) REFERENCES medications (id)
                    )
                """)
                
                cursor.execute("""
                    CREATE TABLE IF NOT EXISTS interactions (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        medication_name TEXT NOT NULL,
                        interaction_type TEXT NOT NULL,
                        warning_message TEXT NOT NULL
                    )
                """)
                
                cursor.execute("""
                    CREATE TABLE IF NOT EXISTS logs (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp REAL NOT NULL,
                        event_type TEXT NOT NULL,
                        details TEXT
                    )
                """)
                
                cursor.execute("SELECT COUNT(*) FROM medications")
                if cursor.fetchone()[0] == 0:
                    self._seed_initial_data(cursor)
                    
                conn.commit()
        except sqlite3.Error:
            pass

    def _seed_initial_data(self, cursor: sqlite3.Cursor) -> None:
        meds = [
            ("ibuprofen", "400mg", 1, 1),
            ("paracetamol", "500mg", 0, 1),
            ("amoxicillin", "500mg", 1, 1)
        ]
        cursor.executemany(
            "INSERT INTO medications (name, dosage, requires_food, requires_water) VALUES (?, ?, ?, ?)",
            meds
        )
        
        interactions = [
            ("ibuprofen", "empty_stomach", "Warning: Taking Ibuprofen on an empty stomach can cause severe gastrointestinal irritation, heartburn, or ulcers. Please eat something first."),
            ("ibuprofen", "alcohol", "Warning: Do not consume alcohol with Ibuprofen, it increases the risk of stomach bleeding.")
        ]
        cursor.executemany(
            "INSERT INTO interactions (medication_name, interaction_type, warning_message) VALUES (?, ?, ?)",
            interactions
        )
        
        schedules = [
            (1, "20:00")
        ]
        cursor.executemany(
            "INSERT INTO schedules (medication_id, time_str) VALUES (?, ?)",
            schedules
        )

    def _register_subscriptions(self) -> None:
        self._bus.subscribe("OBJECT_DETECTED", self._on_object_detected)
        self._bus.subscribe("PATIENT_ATE_MEAL", self._on_patient_ate)
        self._bus.subscribe("PATIENT_DRANK_WATER", self._on_patient_drank)

    def _on_patient_ate(self, data: Any) -> None:
        self._last_meal_time = time.time()
        self._log_event("MEAL", "Patient consumed food")

    def _on_patient_drank(self, data: Any) -> None:
        self._last_water_time = time.time()
        self._log_event("WATER", "Patient consumed water")

    def _on_object_detected(self, data: Any) -> None:
        if not isinstance(data, dict):
            return
            
        obj_name = data.get("object", "").lower()
        if "pill" in obj_name or "bottle" in obj_name or "ibuprofen" in obj_name:
            if "ibuprofen" in obj_name or ("pill" in obj_name and data.get("confidence", 0) > 0.85):
                self._check_clinical_interactions("ibuprofen")

    def _check_clinical_interactions(self, med_name: str) -> None:
        try:
            with sqlite3.connect(self._db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("SELECT requires_food, requires_water FROM medications WHERE name = ?", (med_name,))
                result = cursor.fetchone()
                
                if not result:
                    return
                    
                req_food, req_water = result
                current_time = time.time()
                
                warnings = []
                
                if req_food:
                    hours_since_meal = (current_time - self._last_meal_time) / 3600.0
                    if self._last_meal_time == 0.0 or hours_since_meal > 4.0:
                        cursor.execute(
                            "SELECT warning_message FROM interactions WHERE medication_name = ? AND interaction_type = 'empty_stomach'", 
                            (med_name,)
                        )
                        inter_res = cursor.fetchone()
                        msg = inter_res[0] if inter_res else f"Warning: {med_name} should be taken with food."
                        warnings.append(msg)
                        
                if warnings:
                    for w in warnings:
                        self._bus.publish("COMMAND_SPEAK", {"text": w})
                        self._bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "concerned", "transition": 0.5}})
                        self._log_event("CLINICAL_WARNING", w)
        except sqlite3.Error:
            pass

    def _log_event(self, event_type: str, details: str) -> None:
        try:
            with sqlite3.connect(self._db_path) as conn:
                cursor = conn.cursor()
                cursor.execute(
                    "INSERT INTO logs (timestamp, event_type, details) VALUES (?, ?, ?)",
                    (time.time(), event_type, details)
                )
                conn.commit()
        except sqlite3.Error:
            pass

    async def _trigger_medication_reminder(self, med_name: str, dosage: str, schedule_id: int) -> None:
        self._bus.publish("COMMAND_CPP_CMD", {"cmd": "wake", "data": {"transition": 1.5}})
        await asyncio.sleep(2.0)
        
        self._bus.publish("COMMAND_CPP_CMD", {"cmd": "set_expression", "data": {"type": "curious", "transition": 0.5}})
        
        msg = f"Hello. It is time to take your {med_name}, dosage {dosage}."
        self._bus.publish("COMMAND_SPEAK", {"text": msg})
        self._bus.publish("MEDICATION_TIME", {"medication": med_name, "schedule_id": schedule_id})
        
        self._log_event("REMINDER_SENT", f"Reminded patient to take {med_name} {dosage}")

    def _sync_trigger_medication_reminder(self, med_name: str, dosage: str, schedule_id: int) -> None:
        try:
            loop = asyncio.get_running_loop()
            asyncio.run_coroutine_threadsafe(
                self._trigger_medication_reminder(med_name, dosage, schedule_id), 
                loop
            )
        except RuntimeError:
            asyncio.run(self._trigger_medication_reminder(med_name, dosage, schedule_id))

    def _load_schedules(self) -> None:
        try:
            with sqlite3.connect(self._db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT s.id, m.name, m.dosage, s.time_str 
                    FROM schedules s 
                    JOIN medications m ON s.medication_id = m.id 
                    WHERE m.active = 1
                """)
                
                for row in cursor.fetchall():
                    sched_id, med_name, dosage, time_str = row
                    hour, minute = map(int, time_str.split(':'))
                    
                    self._scheduler.add_job(
                        self._sync_trigger_medication_reminder,
                        'cron',
                        hour=hour,
                        minute=minute,
                        args=[med_name, dosage, sched_id],
                        id=f"med_{sched_id}",
                        replace_existing=True
                    )
        except (sqlite3.Error, ValueError):
            pass

    def start(self) -> None:
        if not self._scheduler.running:
            self._load_schedules()
            self._scheduler.start()

    def stop(self) -> None:
        if self._scheduler.running:
            self._scheduler.shutdown()
