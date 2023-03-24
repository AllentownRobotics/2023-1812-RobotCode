// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  private CANSparkMax wristMotor;
  private AbsoluteEncoder wristEncoder;
  private SparkMaxPIDController wristPIDController;
  private double desiredAngle;
  /** Creates a new Arm. */
  public Wrist() {
    wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setSmartCurrentLimit(40);

    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristEncoder.setInverted(true);
    wristEncoder.setPositionConversionFactor(360.0);
    wristEncoder.setVelocityConversionFactor(wristEncoder.getPositionConversionFactor() / 60.0);

    wristPIDController = wristMotor.getPIDController();
    wristPIDController.setFeedbackDevice(wristEncoder);
    wristPIDController.setP(WristConstants.wristP);
    wristPIDController.setI(WristConstants.wristI);
    wristPIDController.setD(WristConstants.wristD);
    wristPIDController.setOutputRange(WristConstants.wristOutputMax, WristConstants.wristOutputMin);

    wristMotor.burnFlash();
  }
  @Override
  public void periodic() {
    wristPIDController.setReference(desiredAngle, ControlType.kPosition);
    SmartDashboard.putNumber("Arm Angle", wristEncoder.getPosition());
    SmartDashboard.putNumber("Set Point", desiredAngle);
    // This method will be called once per scheduler run
  }
  public double getDesiredAngle()
  {
    return desiredAngle;
  }
  public double getAngle()
  {
    return wristEncoder.getPosition();
  }
  public void setWrist(double angle)
  {
    desiredAngle = angle;
  }

  public boolean atSetpoint()
  {
    return Math.abs(wristEncoder.getPosition()-desiredAngle)<WristConstants.wristAngleAllowance;
  }
}
