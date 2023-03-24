// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class SlowDriveCMD extends CommandBase {
 
  private boolean fieldOriented;
    
  private DriveTrain drive;

  private CommandXboxController drivecontroller;

  public SlowDriveCMD(CommandXboxController controller, boolean fieldOriented, DriveTrain drive) {
      this.drive = drive;
      addRequirements(drive);

      this.fieldOriented = fieldOriented;
      this.drivecontroller = controller;
  }

  @Override
  public void execute() {
      drive.slowDrive(
          MathUtil.applyDeadband(drivecontroller.getLeftY(), 0.3),
          MathUtil.applyDeadband(drivecontroller.getLeftX(), 0.3),
          MathUtil.applyDeadband(-drivecontroller.getRightX(), 0.3),
          fieldOriented);
  }
}
