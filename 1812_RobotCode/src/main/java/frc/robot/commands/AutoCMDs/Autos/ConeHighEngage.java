// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDs.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.commands.AutoCMDs.AutoLevel;
import frc.robot.commands.AutoCMDs.FollowPath;
import frc.robot.commands.AutoCMDs.ResetOdometrytoTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class ConeHighEngage extends SequentialCommandGroup {
    public ConeHighEngage(DriveTrain driveSubsystem, Arm armSubsystem, Claw clawSubsystem, RobotContainer robotContainer)
    {
      addCommands(
        new ResetOdometrytoTrajectory("path", driveSubsystem),
        new FollowPath("path", 0, 0, driveSubsystem).getCommand(),
        new ArmDownCMD(armSubsystem)
      );
    }
}
