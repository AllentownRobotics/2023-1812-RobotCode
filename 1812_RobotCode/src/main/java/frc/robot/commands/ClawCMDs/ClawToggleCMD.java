// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCMDs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClawToggleCMD extends InstantCommand {
  private Claw clawSubsystem;
  public ClawToggleCMD(Claw clawSubsystem) {
    this.clawSubsystem = clawSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.toggleClaw();
  }
}
