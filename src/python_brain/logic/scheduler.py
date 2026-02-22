import logging
import time
from typing import Any, Callable

try:
    from apscheduler.schedulers.background import BackgroundScheduler
    from apscheduler.triggers.cron import CronTrigger
    from apscheduler.triggers.interval import IntervalTrigger
except ImportError:
    BackgroundScheduler = None
    CronTrigger = None
    IntervalTrigger = None

from ..core.event_bus import EventBus
from ..utils.logger import get_logger

class GlobalScheduler:
    def __init__(self, event_bus: EventBus) -> None:
        self.bus = event_bus
        self.logger = get_logger()
        self._scheduler = BackgroundScheduler() if BackgroundScheduler else None
        self._running = False

    def start(self) -> bool:
        if self._running or not self._scheduler:
            return True
            
        try:
            self._scheduler.start()
            self._schedule_core_routines()
            self._running = True
            self.logger.info("Global Asynchronous Scheduler started successfully.")
            return True
        except Exception as e:
            self.logger.error(f"Failed to start Global Scheduler: {e}")
            return False

    def stop(self) -> None:
        if self._running and self._scheduler:
            self._scheduler.shutdown(wait=False)
            self._running = False
            self.logger.info("Global Asynchronous Scheduler stopped.")

    def _schedule_core_routines(self) -> None:
        if not self._scheduler:
            return
            
        self._scheduler.add_job(
            self._fire_event,
            IntervalTrigger(minutes=30),
            args=["ROUTINE_MEDICAL_CHECK", {"reason": "interval"}],
            id="routine_med_check",
            replace_existing=True
        )

        self._scheduler.add_job(
            self._fire_event,
            CronTrigger(hour=21, minute=0),
            args=["EVENING_SUMMARY", {"message": "Time for your evening summary."}],
            id="evening_summary",
            replace_existing=True
        )
        
        self._scheduler.add_job(
            self._fire_event,
            CronTrigger(hour=8, minute=30),
            args=["MORNING_GREETING", {"message": "Good morning. I am scanning your vitals for the day."}],
            id="morning_greeting",
            replace_existing=True
        )

    def _fire_event(self, event_type: str, data: Any) -> None:
        self.logger.info(f"Scheduler triggering scheduled event: {event_type}")
        self.bus.publish(event_type, data)

    def add_custom_job(self, event_type: str, data: Any, trigger: Any, job_id: str) -> None:
        if not self._scheduler:
            return
            
        try:
            self._scheduler.add_job(
                self._fire_event,
                trigger,
                args=[event_type, data],
                id=job_id,
                replace_existing=True
            )
            self.logger.info(f"Added custom job {job_id} for event {event_type}")
        except Exception as e:
            self.logger.error(f"Error adding custom job {job_id}: {e}")

    def remove_job(self, job_id: str) -> None:
        if not self._scheduler:
            return
            
        try:
            self._scheduler.remove_job(job_id)
            self.logger.info(f"Removed custom job {job_id}")
        except Exception:
            pass
