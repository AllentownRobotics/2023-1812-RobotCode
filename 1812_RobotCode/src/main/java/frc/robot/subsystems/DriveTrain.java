// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.FL_DRIVE_ID,
      DriveConstants.FL_TURN_ID,
      DriveConstants.FL_CHASSIS_OFFSET);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.FR_DRIVE_ID,
      DriveConstants.FR_TURN_ID,
      DriveConstants.FR_CHASSIS_OFFSET);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.BL_DRIVE_ID,
      DriveConstants.BL_TURN_ID,
      DriveConstants.BL_CHASSIS_OFFSET);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.BR_DRIVE_ID,
      DriveConstants.BR_TURN_ID,
      DriveConstants.BR_CHASSIS_OFFSET);

  // The gyro sensor
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(GlobalConstants.PIGEON_ID);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveTrain() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Adjust input based on max speed
    xSpeed *= DriveConstants.MAX_SPEED_MPS;
    ySpeed *= DriveConstants.MAX_SPEED_MPS;
    rot *= DriveConstants.MAX_ANGLE_SPEED;
    

    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_MPS);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    if(fieldRelative)
    {
      SmartDashboard.putString("Orientation", "Field Oriented");
    }
    else
    {
      SmartDashboard.putString("Orientation", "Robot Oriented");
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_MPS);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }

  public void levelSet(double speed) {
    m_frontLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
  }
  
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public void turnRobot(double speed)
  {
    m_frontLeft.setDesiredStateNoOpt(new SwerveModuleState(-speed, Rotation2d.fromDegrees(-45)));
    m_frontRight.setDesiredStateNoOpt(new SwerveModuleState(speed, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredStateNoOpt(new SwerveModuleState(-speed, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredStateNoOpt(new SwerveModuleState(speed, Rotation2d.fromDegrees(-45)));
  }

  public void translateRobot(double speed)
  {
    m_frontLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(90)));
    m_frontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(90)));
    m_rearLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(90)));
    m_rearRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(90)));

  }
}
