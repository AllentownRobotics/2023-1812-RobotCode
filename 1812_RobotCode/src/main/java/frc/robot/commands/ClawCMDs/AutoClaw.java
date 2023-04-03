// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

public class AutoClaw extends SequentialCommandGroup {

  public AutoClaw(Claw claw, Wrist wrist) {
  
    addCommands(
      new ClawOpenCMD(claw),
      Commands.waitUntil(claw::pieceInRange),
      new ClawCloseCMD(claw),
      Commands.waitUntil(claw::pieceInClaw));
  }
}
