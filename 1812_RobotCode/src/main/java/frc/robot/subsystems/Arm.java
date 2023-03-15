// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GlobalConstants;

public class Arm extends SubsystemBase {
  private DoubleSolenoid armPiston1;
  private DoubleSolenoid armPiston2;
  /** Creates a new Arm. */
  public Arm() {
    armPiston1 = new DoubleSolenoid(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH, ArmConstants.arm1ForwardChannel, ArmConstants.arm1ReverseChannel);
    armPiston2 = new DoubleSolenoid(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH, ArmConstants.arm2ForwardChannel, ArmConstants.arm2ReverseChannel);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleArm()
  {
    armPiston1.toggle();
    armPiston2.toggle();
  }
}
