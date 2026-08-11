"""
This test module imports tests that come with wpilib, and can be used
to test basic functionality of just about any robot.
"""

from wpilib.testing.robot_tests import (
    test_autonomous,
    test_disabled,
    test_operator_control,
)

__all__ = ["test_autonomous", "test_disabled", "test_operator_control"]
