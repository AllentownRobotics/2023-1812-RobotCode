// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  public NetworkTable table;

  public static double x;
  public static double y;
  public static double ta;
  public static double[] targetRelPos;
  public static double[] RoboRelPos;

  public static boolean tv;

  public double LimePipeline = 1;
  public double April2DPipeline = 0;
  public double April3DPipeline = 9;

  public double purplePipeline = 2;
  public double yellowPipeline = 3;
  public double HumanPlayerAprilTag = 5;
  public double RightShelfPOI = 8;




  public static double currentPipeline;

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    currentPipeline = April2DPipeline;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase LightOn() {
    return runOnce(
        () -> {
          table.getEntry("ledMode").setDouble(3);
        });
  }

  public CommandBase LightOff() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("ledMode").setDouble(1);
        });
  }


  public CommandBase setLimePipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(LimePipeline);
          currentPipeline = LimePipeline;
        });
  }

  public CommandBase setApril2DPipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(April2DPipeline);
          currentPipeline = April2DPipeline;
        });
  }

  public CommandBase setApril3DPipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(April2DPipeline);
          currentPipeline = April3DPipeline;
        });
  }

  public CommandBase setConePipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(yellowPipeline);
          currentPipeline = yellowPipeline;
        });
  }

  public CommandBase setCubePipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(purplePipeline);
          currentPipeline = purplePipeline;
        });
  }

  public CommandBase setHPpipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(HumanPlayerAprilTag);
          currentPipeline = HumanPlayerAprilTag;
        });
  }

  public CommandBase setShelfPipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(RightShelfPOI);
          currentPipeline = RightShelfPOI;
        });
  }

  public CommandBase TapeTracking() {
    return Commands.sequence(LightOn(), setLimePipe());
  }

  public CommandBase April2DTracking() {
    return Commands.sequence(LightOff(), setApril2DPipe());
  }

  public CommandBase April3DTracking() {
    return Commands.sequence(LightOff(), setApril3DPipe());
  }

  public CommandBase ConeTracking() {
    return Commands.sequence(LightOff(), setConePipe());
  }  
  
  public CommandBase CubeTracking() {
    return Commands.sequence(LightOff(), setCubePipe());
  }

  public CommandBase HPStationTracking() {
    return Commands.sequence(LightOff(), setHPpipe());
  }

  public CommandBase ShelfTracking() {
    return Commands.sequence(LightOff(), setShelfPipe());
  }

  public double angleX() {
    return x;
  }

  public double angleY() {
    return y;
  }


  public static double distanceFromTargetMeters() {
    
    if (currentPipeline == 0 || currentPipeline == 5 || currentPipeline ==8) {
      return targetRelPos[2];
    } else if (currentPipeline == 1 || currentPipeline == 3) {
      double LLHeight = 0;
      double TargetHeight = 0;
      double combinedAngle = TargetHeight - y;
      return (TargetHeight - LLHeight)*Math.tan(Units.degreesToRadians(combinedAngle));
    }

     else {return 0;}
  }

  public double[] april3DCords() {
    return table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  }

  public boolean trackingApril() {
    if (table.getEntry("pipeline").getDouble(0.0) == 0) {
      return true;
    } else {return false;}

  }


  public BooleanSupplier targetAquired() {
    return () -> tv;
  }


  public BooleanSupplier tape() {
    if (currentPipeline == 1)
      {return () -> true;}
      else {return () -> false;}
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    try {
      RoboRelPos = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

      targetRelPos = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

      

      x = table.getEntry("tx").getDouble(0.0);
      y = table.getEntry("ty").getDouble(0.0);
      ta = table.getEntry("ta").getDouble(0.0);


      if (table.getEntry("tv").getDouble(0.0) == 1) {
        tv = true;
      } else {tv = false;}
      SmartDashboard.putNumber("Pipeline", currentPipeline);
    } catch (Exception e) {}
  }
}
