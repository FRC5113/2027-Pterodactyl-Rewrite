from lemonlib import LemonInput


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

    def __init__(self, driver: LemonInput, operator: LemonInput):
        self.driver = driver
        self.operator = operator

    def drive_forward(self) -> float:
        return self.driver.getLeftY()

    def drive_strafe(self) -> float:
        return self.driver.getLeftX()

    def drive_rotation(self) -> float:
        return self.driver.getRightX()

    def drive_limit_speed75(self) -> bool:
        return self.driver.getR2Axis() > 0.8

    def drive_limit_speed50(self) -> bool:
        return self.driver.getL2Axis() > 0.8

    def reset_gyro(self) -> bool:
        return self.driver.getSquareButton()

    def intake(self) -> bool:
        return self.operator.getLeftTriggerAxis() > 0.8

    def outtake(self) -> bool:
        return self.operator.getLeftBumper()

    def intake_up(self) -> bool:
        return self.operator.getXButton()

    def intake_down(self) -> bool:
        return self.operator.getBButton()

    def hard_shoot(self) -> bool:
        return self.operator.getAButton()

    def auto_shoot(self) -> bool:
        return self.operator.getRightTriggerAxis() > 0.8

    def unjam(self) -> bool:
        return self.operator.getYButton()


class SingleOI(OI_Base):
    """
    OI for a single driver who controls both driving and mechanisms.
    """

    def __init__(self, controller: LemonInput):
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
        return self.controller.getLeftBumper()

    def intake_up(self) -> bool:
        return self.controller.getXButton()

    def intake_down(self) -> bool:
        return self.controller.getBButton()

    def hard_shoot(self) -> bool:
        return self.controller.getAButton()

    def auto_shoot(self) -> bool:
        return self.controller.getRightTriggerAxis() > 0.8

    def unjam(self) -> bool:
        return self.controller.getYButton()
