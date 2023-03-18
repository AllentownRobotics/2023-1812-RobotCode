// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CompressCMD;
import frc.robot.commands.ArmCMDs.ArmDownCMD;
import frc.robot.commands.ArmCMDs.ArmUpCMD;
import frc.robot.commands.ArmCMDs.AutoArm;
import frc.robot.commands.ClawCMDs.AutoClaw;
import frc.robot.commands.ClawCMDs.ClawCloseCMD;
import frc.robot.commands.ClawCMDs.ClawToggleCMD;
import frc.robot.commands.ComplexCMDs.ClawCloseWristUp;
import frc.robot.commands.ComplexCMDs.LimeLightOn;
import frc.robot.commands.ComplexCMDs.PlaceCMD;
import frc.robot.commands.ComplexCMDs.ResetCMD;
import frc.robot.commands.DriveCMDs.AutoLevel;
import frc.robot.commands.DriveCMDs.DriveCMD;
import frc.robot.commands.DriveCMDs.PseudoNodeTargeting;
import frc.robot.commands.WristCMDs.WristToggleCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Compress;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
  public DriveTrain driveTrain = new DriveTrain();
  private Compress compressor = new Compress();
  private Limelight limelight = new Limelight();
  private Arm arm = new Arm();
  public Claw claw = new Claw();
  private Wrist wrist = new Wrist();
 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER);
  private CommandXboxController operatorController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER);

  HashMap<String, Command> commandsMap = new HashMap<>();
  SwerveAutoBuilder autoBuilder = genrateAutoBuilder();
  private SendableChooser<Command> chooser = new SendableChooser<Command>();
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    populateCommandMap();
    //default commands
    driveTrain.setDefaultCommand(new DriveCMD(driverController, true, driveTrain));
    compressor.setDefaultCommand(new CompressCMD(compressor));
   
    //chooser
    chooser.setDefaultOption("Mid", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Mid", 3.0, 3.0)));
    chooser.addOption("Right", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Right", 3.0, 3.0)));
    chooser.addOption("Left", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Left", 3.0, 3.0)));
    chooser.addOption("placeMidRight", autoBuilder.fullAuto(PathPlanner.loadPathGroup("placeMidRight", 3.0, 3.0)));
    chooser.addOption("PlaceMidLeft", autoBuilder.fullAuto(PathPlanner.loadPathGroup("PlaceMidLeft",3.0 , 3.0)));
    chooser.addOption("placeMidBalance", autoBuilder.fullAuto(PathPlanner.loadPathGroup("placeMidBalance", 3.0, 3.0)));
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
    operatorController.a().onTrue(new ClawToggleCMD(claw));
    operatorController.b().onTrue(new WristToggleCMD(wrist));
    operatorController.y().onTrue(new AutoClaw(claw, wrist));
    operatorController.x().onTrue(new ClawCloseWristUp(claw, wrist));
    operatorController.rightBumper().whileTrue(new LimeLightOn(limelight));
    operatorController.povUp().onTrue(new ArmUpCMD(arm));
    operatorController.povDown().onTrue(new ResetCMD(wrist, claw, arm));
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

  private void populateCommandMap()
  {
    commandsMap.put("autoBalance", new AutoLevel(driveTrain));
    commandsMap.put("placeLow", new PlaceCMD(arm, wrist, claw));
    commandsMap.put("armUp", new ArmUpCMD(arm)); 
    commandsMap.put("armDown", new ArmDownCMD(arm)); 
    commandsMap.put("resetCMD", new ResetCMD(wrist, claw, arm)); 
    commandsMap.put("autoArm", new AutoArm(arm, claw, wrist)); 
  }

  private SwerveAutoBuilder genrateAutoBuilder()
  {
    return new SwerveAutoBuilder(
      driveTrain::getPose,
      driveTrain::resetOdometry,
      DriveConstants.DRIVE_KINEMATICS,
      new PIDConstants(AutoConstants.PX_CONTROLLER, 0, 0),
      new PIDConstants(AutoConstants.P_THETA_CONTROLLER, 0, 0),
      driveTrain::setModuleStates,
      commandsMap,
      true,
      driveTrain);
  }
}
