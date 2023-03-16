// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  private DoubleSolenoid wristPiston1;
  private DoubleSolenoid wristPiston2;
  /** Creates a new Arm. */
  public Wrist() {
    wristPiston1 = new DoubleSolenoid(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH, WristConstants.wrist1ForwardChannel, WristConstants.wrist1ReverseChannel);
    wristPiston2 = new DoubleSolenoid(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH, WristConstants.wrist2ForwardChannel, WristConstants.wrist2ReverseChannel);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setWrist(Value value)
  {
    wristPiston1.set(value);
    wristPiston2.set(value);
  }
}
