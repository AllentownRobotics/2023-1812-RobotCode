// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GlobalConstants;

public class Arm extends SubsystemBase {
  private DoubleSolenoid armPiston;
  /** Creates a new Arm. */
  public Arm() {
    armPiston = new DoubleSolenoid(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH, ArmConstants.armForwardChannel, ArmConstants.armReverseChannel);

    armPiston.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArm(Value value)
  {
    armPiston.set(value);
  }
  public void toggleArm()
  {
    armPiston.toggle();
  }
}
