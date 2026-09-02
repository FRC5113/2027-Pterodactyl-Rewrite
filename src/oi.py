from wpilib import Gamepad, SmartDashboard


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
        return self.driver.getLeftY()

    def drive_strafe(self) -> float:
        return self.driver.getLeftX()

    def drive_rotation(self) -> float:
        return self.driver.getRightX()

    def drive_limit_speed75(self) -> bool:
        return self.driver.getRightTriggerAxis() > 0.8

    def drive_limit_speed50(self) -> bool:
        return self.driver.getLeftTriggerAxis() > 0.8

    def reset_gyro(self) -> bool:
        return self.driver.getWestFaceButton()

    def intake(self) -> bool:
        return self.operator.getLeftTriggerAxis() > 0.8

    def outtake(self) -> bool:
        return self.operator.getLeftBumperButton()

    def intake_up(self) -> bool:
        return self.operator.getWestFaceButton()

    def intake_down(self) -> bool:
        return self.operator.getEastFaceButton()

    def hard_shoot(self) -> bool:
        return self.operator.getSouthFaceButton()

    def auto_shoot(self) -> bool:
        return self.operator.getRightTriggerAxis() > 0.8

    def unjam(self) -> bool:
        return self.operator.getNorthFaceButton()


class SingleOI(OI_Base):
    """
    OI for a single driver who controls both driving and mechanisms.
    """

    def __init__(self, controller: Gamepad):
        self.controller = controller

    def drive_forward(self) -> float:
        return self.controller.getLeftY()

    def drive_strafe(self) -> float:
        return self.controller.getLeftX()

    def drive_rotation(self) -> float:
        return self.controller.getRightX()

    def reset_gyro(self) -> bool:
        return self.controller.getStartButton()

    def intake(self) -> bool:
        return self.controller.getLeftTriggerAxis() > 0.8

    def outtake(self) -> bool:
        return self.controller.getLeftBumperButton()

    def intake_up(self) -> bool:
        return self.controller.getWestFaceButton()

    def intake_down(self) -> bool:
        return self.controller.getEastFaceButton()

    def hard_shoot(self) -> bool:
        return self.controller.getSouthFaceButton()

    def auto_shoot(self) -> bool:
        return self.controller.getRightTriggerAxis() > 0.8

    def unjam(self) -> bool:
        return self.controller.getNorthFaceButton()


class Twitch_OI(OI_Base):
    def drive_forward(self) -> float:
        return SmartDashboard.getNumber("LeftY")

    def drive_strafe(self) -> float:
        return SmartDashboard.getNumber("LeftX")

    def drive_rotation(self) -> float:
        return SmartDashboard.getNumber("RightX")

    def drive_limit_speed75(self) -> bool:
        return False

    def drive_limit_speed50(self) -> bool:
        return False

    def reset_gyro(self) -> bool:
        return False

    def intake(self) -> bool:
        return SmartDashboard.getBoolean("intake")

    def outtake(self) -> bool:
        return False

    def intake_up(self) -> bool:
        return False

    def intake_down(self) -> bool:
        return False

    def hard_shoot(self) -> bool:
        return False

    def auto_shoot(self) -> bool:
        return SmartDashboard.getBoolean("shoot")

    def funny_shoot(self) -> bool:
        return False

    def unjam(self) -> bool:
        return False
