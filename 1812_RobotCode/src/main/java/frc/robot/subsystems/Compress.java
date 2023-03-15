// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants;

public class Compress extends SubsystemBase {
  Compressor comp;
  
  public Compress() {
    comp = new Compressor(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void run()
  {
    SmartDashboard.putNumber("Pressure Switch Value", comp.getPressure());
    comp.enableAnalog(60,120);
  }
}
