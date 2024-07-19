import commands2
import wpilib.drive
from drivetrainsubsys import DriveTrain

class Find_and_Follow_Apriltag(commands2.CommandBase):
    def __init__(self, drivetrain: DriveTrain) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        print ("Starting command to follow Apriltags ")

    def execute(self) -> None:
        print ("Running search")
        no_tag_turn_speed = 0.3
        tag_turn_speed = 0
        forward_speed = 0.0
        turn_rate_reduction = 0.04
        yaw_offset = 0
        yaw_offset_max = 3

        # See if AprilTag in view
        apriltag_present = self.drivetrain.get_Apriltag_status()

        if apriltag_present:
            yaw_offset = self.drivetrain.get_Apriltag_yaw()
            tag_turn_speed = yaw_offset * turn_rate_reduction

            if (abs(yaw_offset) >  yaw_offset_max):
                # Need to move to align
                self.drivetrain.drive_teleop(forward_speed, tag_turn_speed)
                print ("Turning to align: ", yaw_offset )

            else:  # Don't need to move
                self.drivetrain.drive_teleop(forward_speed, 0.0)
                print ("Don't need to move to align")


        # AprilTag not in sight so turn slowly
        else:
            self.drivetrain.drive_teleop(forward_speed, no_tag_turn_speed)   # (forward, turn)
            print ("Searching")


        # print ("Status: ", apriltag_present, "   Turn:  ", yaw_offset)

 
    def end(self, interrupted: bool) -> None:
        self.drivetrain.drive_teleop(0.0, 0.0)    # Stop the robot

    def isFinished(self) -> bool:
        return False

