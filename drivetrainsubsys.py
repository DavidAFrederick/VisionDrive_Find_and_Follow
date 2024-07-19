import wpilib
from commands2 import Subsystem, Command
import wpilib.drive

from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals.spn_enums import (InvertedValue,NeutralModeValue)
from phoenix6.controls import DutyCycleOut
from phoenix6 import StatusCode

import ntcore        ### Vision processing


class DriveTrain(Subsystem):

   def __init__(self) -> None:
       super().__init__()         # Call the parent's (Super) initialization function

       # Apply all the configurations to the left and right side Talon motor controllers
       self.__configure_left_side_drive()
       self.__configure_right_side_drive()
       self.left_encoder_value = 0
       self.right_encoder_value = 0
       self.robot_heading = 0


               ### Vision Processing
       self.NT_table_instance = ntcore.NetworkTableInstance.getDefault()
    #    self.datatable = self.NT_table_instance.getTable("photonvision/USB_Camera_1")                  ### Update
       self.datatable = self.NT_table_instance.getTable("photonvision/limelight")                  ### Update
    #    self.datatable = self.NT_table_instance.getTable("photonvision/Microsoft_LifeCam_HD-3000")       ### Update
       self.camera_has_target = self.datatable.getBooleanTopic("hasTarget").subscribe(0)
       self.yaw_angle_to_target = self.datatable.getFloatTopic("targetYaw").subscribe(0)
       self.NT_table_instance.startClient4("Example Client")
       self.NT_table_instance.setServerTeam(1895)                                          ### Update
    #    self.NT_table_instance.setServer("192.168.6.131")                                     ### Update
       self.NT_table_instance.startDSClient()
       ###

       print ("DriveTrain  Subsystem Initialization complete")


       
   def get_Apriltag_status(self) -> bool:
        return self.camera_has_target.get()

   def get_Apriltag_yaw(self) -> float:
        return self.yaw_angle_to_target.get()


   def __configure_left_side_drive(self) -> None:
       self._left_leader_motor = TalonFX(1)     # 1 is the CAN bus address
       # Applying a new configuration will erase all other config settings since
       # we start with a blank config so each setting needs to be explicitly set
       # here in the config method
       config = TalonFXConfiguration()
       # Set the left side motors to be counter clockwise positive
       config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
       # Set the motors to electrically stop instead of coast
       config.motor_output.neutral_mode = NeutralModeValue.BRAKE
       # Apply the configuration to the motors
       for i in range(6):  # Try 5 times
           ret = self._left_leader_motor.configurator.apply(config)
      
   def __configure_right_side_drive(self) -> None:
       self._right_leader_motor = TalonFX(2)     # 2 is the CAN bus address
       # Applying a new configuration will erase all other config settings since
       # we start with a blank config so each setting needs to be explicitly set
       # here in the config method
       config = TalonFXConfiguration()
       # Set the left side motors to be counter clockwise positive
       config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
       # Set the motors to electrically stop instead of coast
       config.motor_output.neutral_mode = NeutralModeValue.BRAKE
       # Apply the configuration to the motor with a 5 second timeout
       for i in range(6):  # Try 5 times
           ret = self._right_leader_motor.configurator.apply(config)

   ########################### Drivetrain Drive methods #######################

   def drive_teleop(self, forward: float, turn: float):

       reduction = 0.3
       turn = turn * reduction
       forward = forward * reduction

       speeds = wpilib.drive.DifferentialDrive.curvatureDriveIK(forward, turn, True)

       self._left_percent_out: DutyCycleOut = DutyCycleOut(0)
       self._right_percent_out: DutyCycleOut = DutyCycleOut(0)

       self._left_percent_out.output = speeds.left
       self._right_percent_out.output = speeds.right

       self._left_leader_motor.set_control(self._left_percent_out)
       self._right_leader_motor.set_control(self._right_percent_out)


        # In Simulation, encoders count 0 to 16 to cross the field
       self.left_encoder_value = self.left_encoder_value + speeds.left * 0.1
       self.right_encoder_value = self.right_encoder_value + speeds.right * 0.1
       self.robot_heading = self.robot_heading + (turn * 10)

   def get_left_encoder_value(self):
       return self.left_encoder_value

   def get_right_encoder_value(self):
       return self.right_encoder_value

   def get_robot_angle(self):
       return self.robot_heading
   


   def periodic(self) -> None:

       wpilib.SmartDashboard.putBoolean(
            "PhotonVision: AprilTag in sight:", self.camera_has_target.get()
        )
       wpilib.SmartDashboard.putNumber(
            "Yaw Angle to AprilTag", self.yaw_angle_to_target.get()
        )
