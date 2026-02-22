import logging
import os
from logging.handlers import RotatingFileHandler

class SystemLogger:
    _instance = None

    def __new__(cls, log_dir: str = "logs", log_file: str = "baymax.log", max_bytes: int = 50 * 1024 * 1024, backup_count: int = 5):
        if cls._instance is None:
            cls._instance = super(SystemLogger, cls).__new__(cls)
            cls._instance._setup_logger(log_dir, log_file, max_bytes, backup_count)
        return cls._instance

    def _setup_logger(self, log_dir: str, log_file: str, max_bytes: int, backup_count: int) -> None:
        if not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir)
            except OSError:
                pass
                
        self.logger = logging.getLogger("BaymaxCore")
        self.logger.setLevel(logging.DEBUG)
        
        if self.logger.hasHandlers():
            self.logger.handlers.clear()
            
        formatter = logging.Formatter(
            fmt='%(asctime)s | %(levelname)-8s | %(module)-15s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        log_path = os.path.join(log_dir, log_file)
        
        try:
            file_handler = RotatingFileHandler(
                filename=log_path,
                maxBytes=max_bytes,
                backupCount=backup_count,
                encoding='utf-8'
            )
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
        except OSError:
            pass
            
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

def get_logger() -> logging.Logger:
    return SystemLogger().logger
