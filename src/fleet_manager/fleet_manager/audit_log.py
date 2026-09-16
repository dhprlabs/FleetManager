"""Structured, correlation-friendly audit events for fleet decisions."""

import json
from datetime import datetime, timezone
from typing import Any, Dict, Optional


AUDIT_PREFIX = 'FLEET_AUDIT'


def audit_event(logger, component: str, event: str, robot_id: str = '',
                level: str = 'info', trace_id: Optional[str] = None,
                **details: Any) -> Dict[str, Any]:
    """Write one JSON event through ROS logging without risking control flow."""
    record: Dict[str, Any] = {
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'component': component,
        'event': event,
        'robot_id': robot_id,
    }
    if trace_id:
        record['trace_id'] = trace_id
    record.update(details)
    text = f'{AUDIT_PREFIX} {json.dumps(record, sort_keys=True, default=str, separators=(",", ":"))}'
    getattr(logger, level, logger.info)(text)
    return record
