import tunables
from wpilib import Gamepad


class OI_Base:
    """
    Base class for operator interface (OI) that defines the structure of the OI.
    """

    def drive_forward(self) -> float:
        return 0.0

    def drive_strafe(self) -> float:
        return 0.0

    def drive_rotation(self) -> float:
        return 0.0

    def drive_limit_speed75(self) -> bool:
        return False

    def drive_limit_speed50(self) -> bool:
        return False

    def reset_gyro(self) -> bool:
        return False

    def intake(self) -> bool:
        return False

    def outtake(self) -> bool:
        return False

    def intake_up(self) -> bool:
        return False

    def intake_down(self) -> bool:
        return False

    def hard_shoot(self) -> bool:
        return False

    def auto_shoot(self) -> bool:
        return False

    def funny_shoot(self) -> bool:
        return False

    def unjam(self) -> bool:
        return False


class DoubleOI(OI_Base):
    """
    OI for two drivers, one for driving and one for operating the mechanisms.
    """

    def __init__(self, driver: Gamepad, operator: Gamepad):
        self.driver = driver
        self.operator = operator

    def drive_forward(self) -> float:
        return self.driver.get_left_y()

    def drive_strafe(self) -> float:
        return self.driver.get_left_x()

    def drive_rotation(self) -> float:
        return self.driver.get_right_x()

    def drive_limit_speed75(self) -> bool:
        return self.driver.get_right_trigger() > 0.8

    def drive_limit_speed50(self) -> bool:
        return self.driver.get_left_trigger() > 0.8

    def reset_gyro(self) -> bool:
        return self.driver.get_face_left_button()

    def intake(self) -> bool:
        return self.operator.get_left_trigger() > 0.8

    def outtake(self) -> bool:
        return self.operator.get_left_bumper_button()

    def intake_up(self) -> bool:
        return self.operator.get_face_left_button()

    def intake_down(self) -> bool:
        return self.operator.get_face_right_button()

    def hard_shoot(self) -> bool:
        return self.operator.get_face_down_button()

    def auto_shoot(self) -> bool:
        return self.operator.get_right_trigger() > 0.8

    def unjam(self) -> bool:
        return self.operator.get_face_up_button()


class SingleOI(OI_Base):
    """
    OI for a single driver who controls both driving and mechanisms.
    """

    def __init__(self, controller: Gamepad):
        self.controller = controller

    def drive_forward(self) -> float:
        return self.controller.get_left_y()

    def drive_strafe(self) -> float:
        return self.controller.get_left_x()

    def drive_rotation(self) -> float:
        return -self.controller.get_right_x()

    def reset_gyro(self) -> bool:
        return self.controller.get_start_button()

    def intake(self) -> bool:
        return self.controller.get_left_trigger() > 0.8

    def outtake(self) -> bool:
        return self.controller.get_left_bumper_button()

    def intake_up(self) -> bool:
        return self.controller.get_face_left_button()

    def intake_down(self) -> bool:
        return self.controller.get_face_right_button()

    def hard_shoot(self) -> bool:
        return self.controller.get_face_down_button()

    def auto_shoot(self) -> bool:
        return self.controller.get_right_trigger() > 0.8

    def unjam(self) -> bool:
        return self.controller.get_face_up_button()


class Twitch_OI(OI_Base):
    def drive_forward(self) -> float:
        return tunables.add("LeftY", 0.0).get()

    def drive_strafe(self) -> float:
        return tunables.add("LeftX", 0.0).get()

    def drive_rotation(self) -> float:
        return tunables.add("RightX", 0.0).get()

    def drive_limit_speed75(self) -> bool:
        return False

    def drive_limit_speed50(self) -> bool:
        return False

    def reset_gyro(self) -> bool:
        return False

    def intake(self) -> bool:
        return tunables.add("intake", False).get()

    def outtake(self) -> bool:
        return False

    def intake_up(self) -> bool:
        return False

    def intake_down(self) -> bool:
        return False

    def hard_shoot(self) -> bool:
        return False

    def auto_shoot(self) -> bool:
        return tunables.add("shoot", False).get()

    def funny_shoot(self) -> bool:
        return False

    def unjam(self) -> bool:
        return False
