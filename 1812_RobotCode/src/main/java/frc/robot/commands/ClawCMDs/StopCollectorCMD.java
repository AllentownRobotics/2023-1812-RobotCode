// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCMDs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class StopCollectorCMD extends InstantCommand {
  public Claw claw; 
  public StopCollectorCMD(Claw claw) {
    this.claw = claw; 
  }


  @Override
  public void initialize() {
    claw.stopCollector();
  }
}
