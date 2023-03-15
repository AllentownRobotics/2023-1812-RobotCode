// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCMD;
import frc.robot.commands.ClawCMD;
import frc.robot.commands.CompressCMD;
import frc.robot.commands.WristCMD;
import frc.robot.commands.DriveCMDs.AutoLevel;
import frc.robot.commands.DriveCMDs.DriveCMD;
import frc.robot.commands.DriveCMDs.PseudoNodeTargeting;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Compress;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Compress compressor = new Compress();
  private final Limelight limelight = new Limelight();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Wrist wirst = new Wrist();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER);
  private final CommandXboxController operatorController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER);

  private SendableChooser<Command> chooser = new SendableChooser<Command>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //default commands
    driveTrain.setDefaultCommand(new DriveCMD(driverController, false, driveTrain));
    compressor.setDefaultCommand(new CompressCMD(compressor));
    //chooser
    chooser.setDefaultOption("Auto", null);
    chooser.addOption("Auto", null);
    chooser.addOption("Auto", null);
    SmartDashboard.putData("Auto Chooser", chooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //drive controller configs
    driverController.x().whileTrue(new AutoLevel(driveTrain));
    driverController.rightBumper().whileTrue(new RunCommand(() -> driveTrain.setX(), driveTrain));
    driverController.start().onTrue(new InstantCommand(() -> driveTrain.zeroHeading(), driveTrain));
    driverController.povUp().onTrue(limelight.April2DTracking());
    driverController.povDown().onTrue(limelight.TapeTracking());
    driverController.leftTrigger().whileTrue(new PseudoNodeTargeting(driveTrain, driverController));

    //operator controller configs
    operatorController.a().onTrue(new ArmCMD(arm));
    operatorController.b().onTrue(new ClawCMD(claw));
    operatorController.x().onTrue(new WristCMD(wirst));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }
}
