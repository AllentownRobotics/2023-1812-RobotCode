// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Controllers
  public static class OIConstants {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
  }

  public static class GlobalConstants{
    //Pneumatics
  public static final int PNEUMATICS_ID = 14;
    //Gyro ID
  public static final int PIGEON_ID = 9;
  }
  //DriveTrain
  public static final class DriveConstants{
  
  //Drive parameters 
  public static final double MAX_SPEED_MPS = 4;
  public static final double MAX_ANGLE_SPEED = 2 * Math.PI; //Radians per sec
  
  //chassis config
  public static final double TRACK_WIDTH = Units.inchesToMeters(26); 
  public static final double WHEEL_BASE = Units.inchesToMeters(26);
  
  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE /2, -TRACK_WIDTH /2));
  
  //angular offsets of modules from chassis
  public static final double FL_CHASSIS_OFFSET = -Math.PI / 2;
  public static final double FR_CHASSIS_OFFSET = 0.0; 
  public static final double BL_CHASSIS_OFFSET = Math.PI; 
  public static final double BR_CHASSIS_OFFSET = Math.PI / 2;
  
  //SPARK CAN IDS
  /*driving motor ids */
  public static final int FL_DRIVE_ID = 1;
  public static final int BL_DRIVE_ID = 7;
  public static final int FR_DRIVE_ID = 3;
  public static final int BR_DRIVE_ID = 5; 
  
  /*turning motors ids */
  public static final int FL_TURN_ID = 2; 
  public static final int BL_TURN_ID = 8; 
  public static final int FR_TURN_ID = 4; 
  public static final int BR_TURN_ID = 6;
  
  public static final boolean GYRO_REVERSED = false;
  
  
  }
  
  public static final class NeoMotorConstants {
      public static final double NEO_FREE_SPEED = 5676;
    }
  
    public static final class ModuleConstants{
      /*pinion gear teeth */    
      public static final int DRIVE_MOTOR_TEETH = 14;  
  
      public static final boolean TURN_ENCODER_INVERTED = true;
  
  //Calculations for drive motor conversion factors and feed forwards
      public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.NEO_FREE_SPEED / 60;
      public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(2.85);
      public static final double WHEEL_CIRCUMFRENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
      public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_TEETH * 15);
      public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFRENCE_METERS) / DRIVE_MOTOR_REDUCTION;
  
      public static final double DRIVE_ENCODER_POS_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
      / DRIVE_MOTOR_REDUCTION; // meters
      public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
      / DRIVE_MOTOR_REDUCTION) / 60.0; // meters per second
  
      public static final double TURN_ENCODER_POS_FACTOR = (2 * Math.PI); // radians
      public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second
  
      public static final double TURN_ENCODER_POS_MIN_INPUT = 0; // radians
      public static final double TURN_ENCODER_POS_MAX_INPUT = TURN_ENCODER_POS_FACTOR; // radians
  
      //drive motor PID
      public static final double DRIVE_P = 0.04;
      public static final double DRIVE_I = 0;
      public static final double DRIVE_D = 0;
      public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
      public static final double DRIVE_MIN_OUTPUT = -1;
      public static final double DRIVE_MAX_OUTPUT = 1;
  
      //turn motor PID
      public static final double TURN_P = 1;
      public static final double TURN_I = 0;
      public static final double TURN_D = 0;
      public static final double TURN_FF = 0;
      public static final double TURN_MIN_OUTPUT = -1;
      public static final double TURN_MAX_OUTPUT = 1;
  
      public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kBrake;
      public static final IdleMode TURN_MOTOR_IDLE_MODE = IdleMode.kBrake;
  
      public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50; // amps
      public static final int TURN_MOTOR_CURRENT_LIMIT = 20; // amps
  }
  
  //Auto Constnts 
  public static class AutoConstants{
      
     public static final double AUTO_MAX_SPEED_MPS = 2;
      public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 2;
      public static final double MAX_ANGULAR_SPEED_RPS = Math.PI;
      public static final double MAX_ANGULAR_SPEED_RPS_SQUARED = Math.PI;
   
      public static final double PX_CONTROLLER = 1;
      public static final double PY_CONTROLLER = 1;
      public static final double P_THETA_CONTROLLER = 1; 
      // Constraint for the motion profiled robot angle controller
      public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPS_SQUARED);        
  }

  //Arm Constants
  public static class ArmConstants
  {
    public static final int armForwardChannel = 0;
    public static final int armReverseChannel = 1;
    public static final double armOutSeconds = 0.0;
  }

  //Wrist Constants
  public static class WristConstants
  {
    public static final int wristForwardChannel = 3;
    public static final int wristReverseChannel = 2;
    public static final double wristOutSeconds = 0.0;
    public static final double wristInSeconds = 0.0;
  }

  //Claw Constants
  public static class ClawConstants
  {
    public static final int clawForwardChannel = 5;
    public static final int clawReverseChannel = 4;
    public static final double sensorDistance = 13.5;
    public static final double sensorFluff = 1.0;
    public static final double clawCloseSeconds = 0.0;
  }
}
