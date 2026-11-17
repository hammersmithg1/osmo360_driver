"""ROS-independent camera owner; ROS adapters are imported explicitly."""

from .runtime import OsmoCamera

__all__ = ['OsmoCamera']
