// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.GlobalConstants;

public class Claw extends SubsystemBase {
  private DoubleSolenoid clawPiston;
  public Rev2mDistanceSensor distanceSensor;
  private CANSparkMax rightClawMotor; 
  private CANSparkMax leftClawMotor; 
  private MotorControllerGroup clawMotors;
  //private Rev2mDistanceSensor distanceSensor;
  /** Creates a new Arm. */
  public Claw() {
    clawPiston = new DoubleSolenoid(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH, ClawConstants.clawForwardChannel, ClawConstants.clawReverseChannel);
    clawPiston.set(Value.kForward);
    
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    distanceSensor.setAutomaticMode(true);
    distanceSensor.setEnabled(true);
    distanceSensor.setDistanceUnits(Unit.kInches);
    
    rightClawMotor = new CANSparkMax(ClawConstants.rightClawMotorID, MotorType.kBrushless);
    leftClawMotor = new CANSparkMax(ClawConstants.leftClawMotorID, MotorType.kBrushless);

    rightClawMotor.setIdleMode(IdleMode.kBrake); 
    leftClawMotor.setIdleMode(IdleMode.kBrake); 

    leftClawMotor.setInverted(true);

    clawMotors = new MotorControllerGroup(rightClawMotor, leftClawMotor);
  }
  @Override
  public void periodic() {

    SmartDashboard.putNumber("Distance", distanceSensor.getRange());
  
  }

  public void setClaw(Value value)
  {
    clawPiston.set(value);
  }
  public void toggleClaw()
  {
    clawPiston.toggle();
  }
  public boolean pieceInRange()
  {
    return Math.abs(distanceSensor.getRange()-ClawConstants.sensorCloseDistance)<ClawConstants.sensorCloseAllowance;
  }
  public boolean pieceInClaw()
  {
    return Math.abs(distanceSensor.getRange()-ClawConstants.sensorCollectedDistance)<ClawConstants.sensorCollectedAllowance;
  }
  public void setClawMotors(double speed)
  {
    clawMotors.set(speed);
  }
}
