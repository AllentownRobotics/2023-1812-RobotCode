// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCMDs.ClawOpenCMD;
import frc.robot.commands.ComplexCMDs.ResetCMD;
import frc.robot.commands.WristCMDs.WristDownCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoArm extends SequentialCommandGroup {
  /** Creates a new AutoArm. */
  public AutoArm(Arm arm, Claw claw, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ArmUpCMD(arm), 
    Commands.waitSeconds(4), 
    new WristDownCMD(wrist), 
    Commands.waitSeconds(0.5),
    new ClawOpenCMD(claw), 
    Commands.waitSeconds(0.25), 
    new ResetCMD(wrist, claw, arm)
    
    );
  }
}
