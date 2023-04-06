// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCMDs.ArmDownCMD;
import frc.robot.commands.ClawCMDs.ClawCloseCMD;
import frc.robot.commands.WristCMDs.WristResetCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetCMD extends SequentialCommandGroup {
  /** Creates a new PlaceCMD. */
  public ResetCMD(Wrist wrist, Claw claw, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristResetCMD(wrist),
      Commands.waitSeconds(0.05),
      new ClawCloseCMD(claw),
      new ArmDownCMD(arm));
  }
}
