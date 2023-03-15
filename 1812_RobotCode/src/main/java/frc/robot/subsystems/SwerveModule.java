// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private final CANSparkMax turningSparkMax;
  private final CANSparkMax driveSparkMax;
 
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivePIDController;
  private final SparkMaxPIDController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

public SwerveModule(int driveID, int turningID, double chassisAngularOffset) {

  //set up swerve modules
  driveSparkMax = new CANSparkMax(driveID, MotorType.kBrushless);
  turningSparkMax = new CANSparkMax(turningID, MotorType.kBrushless);
  
  driveSparkMax.restoreFactoryDefaults(); 
  turningSparkMax.restoreFactoryDefaults(); 

  driveEncoder = driveSparkMax.getEncoder();
  turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

  drivePIDController = driveSparkMax.getPIDController();
  turningPIDController = turningSparkMax.getPIDController();

  drivePIDController.setFeedbackDevice(driveEncoder);
  turningPIDController.setFeedbackDevice(turningEncoder);

  driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POS_FACTOR);
  driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR); 
  turningEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
  turningEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);

  turningEncoder.setInverted(ModuleConstants.TURN_ENCODER_INVERTED);

  turningPIDController.setPositionPIDWrappingEnabled(true);
  turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);
  turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
  
  //set drive motor PID values
  drivePIDController.setP(ModuleConstants.DRIVE_P);
  drivePIDController.setI(ModuleConstants.DRIVE_I);
  drivePIDController.setD(ModuleConstants.DRIVE_D);
  drivePIDController.setFF(ModuleConstants.DRIVE_FF);
  drivePIDController.setOutputRange(ModuleConstants.DRIVE_MIN_OUTPUT, ModuleConstants.DRIVE_MAX_OUTPUT);

  //Set turn motor PID values 
  turningPIDController.setP(ModuleConstants.TURN_P);
  turningPIDController.setI(ModuleConstants.TURN_I);
  turningPIDController.setD(ModuleConstants.TURN_D);
  turningPIDController.setFF(ModuleConstants.TURN_FF);
  turningPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);
  
//limits and brakes
driveSparkMax.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
turningSparkMax.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE); 
driveSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

//saves configs of modules in case of brownout
driveSparkMax.burnFlash();
turningSparkMax.burnFlash();

this.chassisAngularOffset = chassisAngularOffset;
desiredState.angle = new Rotation2d(turningEncoder.getPosition());
driveEncoder.setPosition(0);
}

//returns the current state of the module
public SwerveModuleState getState(){

  return new SwerveModuleState(driveEncoder.getVelocity(), 
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));

}

//returns current position of the module 
public SwerveModulePosition getPosition(){
  return new SwerveModulePosition(
    driveEncoder.getPosition(),
    new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
}

//sets desired state for module 
public void setDesiredState(SwerveModuleState swerveModuleStates){

  //apply chassis angular offset to desired state 
  SwerveModuleState correctedDesiredState = new SwerveModuleState();
  correctedDesiredState.speedMetersPerSecond = swerveModuleStates.speedMetersPerSecond;
  correctedDesiredState.angle = swerveModuleStates.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

  //optimize refrences state to eliminate rotation more than 90 degrees
  SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
       new Rotation2d(turningEncoder.getPosition()));

  //set drive and turning sparks to their setpoints
  drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
  turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

  desiredState = optimizedDesiredState;
}
public void setDesiredStateNoOpt(SwerveModuleState desiredState) {

  SwerveModuleState correctedDesiredState = new SwerveModuleState();
  correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
  correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

  


  // Command driving and turning SPARKS MAX towards their respective setpoints.
  drivePIDController.setReference(correctedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
  turningPIDController.setReference(correctedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

  correctedDesiredState = desiredState;
}

//zeros all swerve modules encoders 
public void resetEncoders() {
  driveEncoder.setPosition(0);
}
}
