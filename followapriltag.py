import commands2
import wpilib.drive
from drivetrainsubsys import DriveTrain

class FollowAprilTag(commands2.CommandBase):
    def __init__(self, drivetrain: DriveTrain) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        print ("Starting command to follow Apriltags ")

    def execute(self) -> None:
        
        apriltag_present = self.drivetrain.get_Apriltag_status()
        turn = self.drivetrain.get_Apriltag_yaw() / 25

        print ("Status: ", self.drivetrain.get_Apriltag_status(), "   Turn:  ", self.drivetrain.get_Apriltag_yaw())
        wpilib.SmartDashboard.putNumber(  "Autonomous turn", turn   )

        if apriltag_present:
            forward_speed = 0.0
            # self.drivetrain.drive_teleop(0.1, turn)    # (forward , turn) 
            self.drivetrain.drive_teleop(forward_speed, turn)    # (forward , turn) 
        else:
            self.drivetrain.drive_teleop(0.0, 0.0)
 
    def end(self, interrupted: bool) -> None:
        self.drivetrain.drive_teleop(0.0, 0.0)    # Stop the robot

    def isFinished(self) -> bool:
        return False

