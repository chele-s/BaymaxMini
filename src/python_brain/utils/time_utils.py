import time
from datetime import datetime
from typing import Optional

def current_timestamp_us() -> int:
    return int(time.time() * 1_000_000)

def timestamp_to_conversational(timestamp_s: float, timezone_offset_hours: int = 0) -> str:
    dt = datetime.fromtimestamp(timestamp_s)
    
    hour = dt.hour
    minute = dt.minute
    
    hour += timezone_offset_hours
    if hour >= 24:
        hour -= 24
    elif hour < 0:
        hour += 24
        
    period = "AM"
    if hour >= 12:
        period = "PM"
        if hour > 12:
            hour -= 12
    if hour == 0:
        hour = 12
        
    min_str = f"{minute:02d}" if minute > 0 else "o'clock"
    
    return f"{hour} {min_str} {period}"

def time_since_conversational(past_timestamp_s: float, current_timestamp_s: Optional[float] = None) -> str:
    if current_timestamp_s is None:
        current_timestamp_s = time.time()
        
    diff_seconds = current_timestamp_s - past_timestamp_s
    
    if diff_seconds < 60:
        return "just now"
        
    minutes = int(diff_seconds / 60)
    if minutes < 60:
        if minutes == 1:
            return "1 minute ago"
        return f"{minutes} minutes ago"
        
    hours = int(minutes / 60)
    remain_minutes = minutes % 60
    
    hr_str = "1 hour" if hours == 1 else f"{hours} hours"
    
    if remain_minutes == 0:
        return f"{hr_str} ago"
        
    min_str = "1 minute" if remain_minutes == 1 else f"{remain_minutes} minutes"
    return f"{hr_str} and {min_str} ago"
