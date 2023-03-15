// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class ClassicalTargeting extends CommandBase {
  private boolean fieldOriented = true;
    
  private DriveTrain drive;

  private SlewRateLimiter strafe = new SlewRateLimiter(5);
  private SlewRateLimiter translate = new SlewRateLimiter(5);
  private CommandXboxController controller;
  private PIDController kturningPID = new PIDController(0.02, 0, 0);

  public ClassicalTargeting(CommandXboxController controller, boolean fieldOriented, DriveTrain s_Swerve) {
      this.drive = drive;
      addRequirements(drive);

      this.fieldOriented = fieldOriented;
      this.controller = controller;
  }

  @Override
  public void execute() {
      drive.drive(
          translate.calculate(MathUtil.applyDeadband(-controller.getLeftY(), 0.3)),
          strafe.calculate(MathUtil.applyDeadband(-controller.getLeftX(), 0.3)),
          kturningPID.calculate(Limelight.x, 0),
          fieldOriented);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
