// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.WristCMDs.WristDownCMD;
import frc.robot.commands.WristCMDs.WristUpCMD;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

public class AutoClaw extends SequentialCommandGroup {

  public AutoClaw(Claw claw, Wrist wrist) {
  
    addCommands(
      new WristDownCMD(wrist),
      Commands.waitSeconds(WristConstants.wristOutSeconds),
      new ClawOpenCMD(claw),
      new RunCollectorCMD(claw),
      Commands.waitSeconds(.2),
      new WaitUntilCommand(claw::pieceInRange),
      new StopCollectorCMD(claw),
      Commands.waitSeconds(0.5),
      new ClawCloseCMD(claw), 
      Commands.waitSeconds(0.25),
      new WristUpCMD(wrist));
  }
}
