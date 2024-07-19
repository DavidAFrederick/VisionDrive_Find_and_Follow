import wpilib

# Support for simulation on the field
from wpilib import SmartDashboard, Field2d 
from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Pose2d, Rotation2d

from drivetrainsubsys import DriveTrain
from teleopdrivecmd import TeleopDrive
from followapriltag import FollowAprilTag                 ###  Vision Processing
from find_and_follow_apriltag import Find_and_Follow_Apriltag

from commands2 import Command, RunCommand, TimedCommandRobot, button


class MyRobot(TimedCommandRobot):
  def robotInit(self):
       """
       This function is called upon program startup and
       should be used for any initialization code.
       """

       # Allow the display of the field in the simulation - - - - - - - - - - - - - - - - - - - - - 
       self.field = Field2d()
       SmartDashboard.putData("Field", self.field)
       # Create the Odometry tracker
       self.robot_heading_angle = Rotation2d()
       self.robot_pose = Pose2d()
       self.odometry: DifferentialDriveOdometry = DifferentialDriveOdometry(self.robot_heading_angle, 0, 0)
       self.simulated_left_encoder_distance = 0
       self.simulated_right_encoder_distance = 0

      # Instantiate (create) subsystems - - - - - - - - - - - - - - - - - - - - - 
       self.controller = wpilib.Joystick(0)
       self.drivetrainSubSys: DriveTrain = DriveTrain()

       # Set up commands and bind buttons to commands
       self.drivetrainSubSys.setDefaultCommand( TeleopDrive(self.drivetrainSubSys, self.controller ) )
       button.JoystickButton(self.controller,1).whileTrue(   FollowAprilTag(self.drivetrainSubSys)  )
       button.JoystickButton(self.controller,2).whileTrue(   Find_and_Follow_Apriltag(self.drivetrainSubSys)  )

  def getAutonomousCommand(self) -> Command:
    return Find_and_Follow_Apriltag(self.drivetrainSubSys)          ###  Vision Processing

  def autonomousInit(self):
      """This function is run once each time the robot enters autonomous mode."""
      self._auto_command = self.getAutonomousCommand()

      if self._auto_command is not None:
            self._auto_command.schedule()
      print ("Autonomous Initialization (autonomousInit) Completed ")


  def autonomousPeriodic(self):
      """This function is called periodically during autonomous."""


  def teleopInit(self):
      """This function is called once each time the robot enters teleoperated mode."""
      print ("TeleOpInit Initialization (teleopInit) Completed ")


  def teleopPeriodic(self):
      """This function is called periodically during teleoperated mode."""

      # Update Display of Simulation:
      # Create a robot pose (X,Y position on field and robot heading) from wheel encoders and fake heading - - - - - - - - - 
      self.simulated_left_encoder_distance = self.drivetrainSubSys.get_left_encoder_value()
      self.simulated_right_encoder_distance = self.drivetrainSubSys.get_right_encoder_value()
      self.robot_heading_angle = self.robot_heading_angle.fromDegrees(self.drivetrainSubSys.get_robot_angle())

      self.robot_pose = self.odometry.update(self.robot_heading_angle, 
                                             self.simulated_left_encoder_distance, 
                                             self.simulated_right_encoder_distance)
      self.field.setRobotPose(self.odometry.getPose())

    #   print ("Angle:  %4.2f    Encoder:  L: %4.2f   R: %4.2f " % 
    #          (self.drivetrainSubSys.get_robot_angle(), 
    #           self.simulated_left_encoder_distance, 
    #           self.simulated_right_encoder_distance))
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -     
