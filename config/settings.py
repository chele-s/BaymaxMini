import os
import yaml
from typing import Any, Dict

class Settings:
    _instance = None
    _config: Dict[str, Any] = {}

    def __new__(cls, config_path: str = "config/settings.yaml"):
        if cls._instance is None:
            cls._instance = super(Settings, cls).__new__(cls)
            cls._instance._load_config(config_path)
        return cls._instance

    def _load_config(self, filepath: str) -> None:
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Configuration file {filepath} not found.")
            
        with open(filepath, 'r') as f:
            self._config = yaml.safe_load(f) or {}

    def get(self, path: str, default: Any = None) -> Any:
        keys = path.split('.')
        val = self._config
        for key in keys:
            if isinstance(val, dict) and key in val:
                val = val[key]
            else:
                return default
        return val

CONFIG = Settings()
