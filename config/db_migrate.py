import sqlite3
import logging

class DBMigrator:
    def __init__(self, db_path: str = "config/medicines.db") -> None:
        self.db_path = db_path
        self._logger = logging.getLogger(__name__)

    def _get_current_version(self, cursor: sqlite3.Cursor) -> int:
        try:
            cursor.execute("SELECT version FROM schema_version")
            row = cursor.fetchone()
            return row[0] if row else 0
        except sqlite3.OperationalError:
            cursor.execute("CREATE TABLE schema_version (version INTEGER PRIMARY KEY)")
            cursor.execute("INSERT INTO schema_version (version) VALUES (0)")
            return 0

    def _set_version(self, cursor: sqlite3.Cursor, version: int) -> None:
        cursor.execute("UPDATE schema_version SET version = ?", (version,))

    def migrate_v1(self, cursor: sqlite3.Cursor) -> None:
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS patients (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                age INTEGER,
                weight_kg REAL,
                created_at REAL NOT NULL
            )
        """)
        cursor.execute("ALTER TABLE logs ADD COLUMN pain_level_estimate INTEGER DEFAULT 0")

    def run_migrations(self) -> None:
        try:
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                current_version = self._get_current_version(cursor)
                
                if current_version < 1:
                    try:
                        self.migrate_v1(cursor)
                    except sqlite3.OperationalError:
                        pass
                        
                    self._set_version(cursor, 1)
                    conn.commit()
                    
        except sqlite3.Error:
            pass

if __name__ == "__main__":
    migrator = DBMigrator()
    migrator.run_migrations()
