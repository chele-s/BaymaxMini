from __future__ import annotations

import sqlite3
from pathlib import Path

_SCHEMA_VERSION = 3

_DDL: list[str] = [
    """
    CREATE TABLE IF NOT EXISTS schema_version (
        version     INTEGER NOT NULL
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS patients (
        id          INTEGER PRIMARY KEY,
        name        TEXT    NOT NULL,
        birth_date  TEXT    NOT NULL,
        weight_kg   REAL,
        allergies   TEXT,
        notes       TEXT,
        created_at  TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now')),
        updated_at  TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS medicines (
        id              INTEGER PRIMARY KEY,
        name            TEXT    NOT NULL UNIQUE,
        active_compound TEXT    NOT NULL,
        form            TEXT    NOT NULL CHECK(form IN ('pill','liquid','injection','patch','inhaler')),
        strength_mg     REAL    NOT NULL CHECK(strength_mg > 0),
        unit            TEXT    NOT NULL DEFAULT 'mg',
        contraindications TEXT,
        interactions    TEXT,
        notes           TEXT
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS schedules (
        id              INTEGER PRIMARY KEY,
        patient_id      INTEGER NOT NULL REFERENCES patients(id) ON DELETE CASCADE,
        medicine_id     INTEGER NOT NULL REFERENCES medicines(id) ON DELETE CASCADE,
        dose_mg         REAL    NOT NULL CHECK(dose_mg > 0),
        cron_expr       TEXT    NOT NULL,
        start_date      TEXT    NOT NULL,
        end_date        TEXT,
        with_food       INTEGER NOT NULL DEFAULT 0 CHECK(with_food IN (0,1)),
        active          INTEGER NOT NULL DEFAULT 1 CHECK(active IN (0,1)),
        created_at      TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS dose_log (
        id              INTEGER PRIMARY KEY,
        schedule_id     INTEGER NOT NULL REFERENCES schedules(id) ON DELETE CASCADE,
        due_at          TEXT    NOT NULL,
        taken_at        TEXT,
        status          TEXT    NOT NULL DEFAULT 'pending'
                                CHECK(status IN ('pending','taken','missed','skipped')),
        notes           TEXT
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS vitals_history (
        id              INTEGER PRIMARY KEY,
        patient_id      INTEGER NOT NULL REFERENCES patients(id) ON DELETE CASCADE,
        recorded_at     TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now')),
        heart_rate_bpm  REAL,
        spo2_pct        REAL,
        skin_temp_c     REAL,
        ambient_temp_c  REAL,
        distance_mm     REAL,
        bus_voltage_v   REAL,
        current_ma      REAL,
        battery_pct     REAL,
        alert_level     TEXT    NOT NULL DEFAULT 'none'
                                CHECK(alert_level IN ('none','info','warning','critical'))
    )
    """,
    "CREATE INDEX IF NOT EXISTS idx_dose_log_schedule   ON dose_log(schedule_id, due_at)",
    "CREATE INDEX IF NOT EXISTS idx_dose_log_status     ON dose_log(status)",
    "CREATE INDEX IF NOT EXISTS idx_vitals_patient_time ON vitals_history(patient_id, recorded_at)",
    "CREATE INDEX IF NOT EXISTS idx_schedules_active    ON schedules(active, patient_id)",
]


def _get_version(conn: sqlite3.Connection) -> int:
    row = conn.execute("SELECT version FROM schema_version LIMIT 1").fetchone()
    return row[0] if row else 0


def migrate(db_path: str | Path) -> None:
    path = Path(db_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with sqlite3.connect(path) as conn:
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA foreign_keys=ON")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA temp_store=MEMORY")
        conn.execute("PRAGMA cache_size=-16000")

        current = _get_version(conn)
        if current >= _SCHEMA_VERSION:
            return

        for stmt in _DDL:
            conn.execute(stmt)

        if current == 0:
            conn.execute("DELETE FROM schema_version")
            conn.execute("INSERT INTO schema_version VALUES (?)", (_SCHEMA_VERSION,))
        else:
            conn.execute("UPDATE schema_version SET version=?", (_SCHEMA_VERSION,))

        conn.commit()


if __name__ == "__main__":
    import sys
    target = sys.argv[1] if len(sys.argv) > 1 else "config/medicines.db"
    migrate(target)
    print(f"migrated: {target} → v{_SCHEMA_VERSION}")
