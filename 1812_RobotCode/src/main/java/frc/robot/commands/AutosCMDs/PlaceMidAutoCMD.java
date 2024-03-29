// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCMDs.ArmUpCMD;
import frc.robot.commands.ClawCMDs.ClawOpenCMD;
import frc.robot.commands.ComplexCMDs.ResetCMD;
import frc.robot.commands.WristCMDs.WristShelfCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMidAutoCMD extends SequentialCommandGroup {
  /** Creates a new PlaceCMD. */
  public PlaceMidAutoCMD(Arm arm, Wrist wrist, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmUpCMD(arm),
      Commands.waitSeconds(ArmConstants.armUpSeconds),
      new WristShelfCMD(wrist),
      Commands.waitUntil(wrist::atPlace),
      new ClawOpenCMD(claw),
      Commands.waitSeconds(.5),
      new ResetCMD(wrist, claw, arm));
  }
}
