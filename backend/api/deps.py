# backend/api/deps.py
"""
Dependency injection for API routes.
"""
from functools import lru_cache
from ..core import SharedState

# Global shared state instance
_shared: SharedState = None


def get_shared() -> SharedState:
    """Get the global shared state."""
    global _shared
    if _shared is None:
        _shared = SharedState()
    return _shared


def set_shared(shared: SharedState):
    """Set the global shared state (called during app startup)."""
    global _shared
    _shared = shared


def get_core():
    """Get the experiment capture core from shared state."""
    return get_shared().core
