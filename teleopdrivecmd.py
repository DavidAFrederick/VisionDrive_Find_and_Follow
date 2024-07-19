import wpilib
import wpilib.drive
from commands2 import Command
from drivetrainsubsys import DriveTrain


class TeleopDrive(Command):
   def __init__(self, drivetrain: DriveTrain, controller: wpilib.Joystick):
       self.drivetrain = drivetrain
       self.addRequirements(self.drivetrain)
       self.controller = controller
       print ("TeleOpDrive Command Instantiated")


   def initialize(self):
       print ("TeleOpDrive Command Initialized")


   def execute(self):
       self.drivetrain.drive_teleop(-self.controller.getRawAxis(1),
                                    -self.controller.getRawAxis(0))


   def isFinished(self) -> bool:
       return False
  
   def end(self, interrupted: bool):
       self.drivetrain.drive_teleop(0,0)