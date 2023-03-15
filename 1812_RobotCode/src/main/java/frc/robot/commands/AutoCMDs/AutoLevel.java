// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoLevel extends CommandBase {
        
  private DriveTrain s_Swerve;
  PIDController kLevelingPID = new PIDController(0.035, 0, 0);

  
  public AutoLevel(DriveTrain s_Swerve) {
      this.s_Swerve = s_Swerve;
      addRequirements(s_Swerve);
      kLevelingPID.setIntegratorRange(-3, 3);
      //kLevelingPID.setTolerance(0.5);

  }

  @Override
  public void execute() {
      s_Swerve.levelSet(-kLevelingPID.calculate(s_Swerve.getRoll(), 0));
  }

  @Override
  public boolean isFinished() {
      return false;
  }

}
