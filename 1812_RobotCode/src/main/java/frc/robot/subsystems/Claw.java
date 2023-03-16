// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.GlobalConstants;

public class Claw extends SubsystemBase {
  private DoubleSolenoid clawPiston;
  //private Rev2mDistanceSensor distanceSensor;
  /** Creates a new Arm. */
  public Claw() {
    clawPiston = new DoubleSolenoid(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH, ClawConstants.clawForwardChannel, ClawConstants.clawReverseChannel);
    //distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClaw(Value value)
  {
    clawPiston.set(value);
  }

  /*public BooleanSupplier getDistance()
  {
    return () -> (Math.abs(distanceSensor.getRange()-ClawConstants.distanceCollectThreshold)<.5);
  }*/
}
