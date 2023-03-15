package frc.robot.commands.DriveCMDs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class PseudoNodeTargeting extends SequentialCommandGroup {
    
    private DriveTrain m_drive;
    private CommandXboxController controller;
    private PIDController kturningPID = new PIDController(0.015, 0, 0.0001);
    private PIDController kStrafingPID = new PIDController(0.02, 0, 0.000);
    private PIDController kDrivingPID = new PIDController(0.02, 0, 0);

    private double tyHighCone = 0;
    private double tyHighCube = 0;
    private double tyMidCone = 0;
    private double tyMidCube = 0;
    private double yTarget = 0;
    private SlewRateLimiter translate = new SlewRateLimiter(5);

    public PseudoNodeTargeting(DriveTrain m_drive, CommandXboxController controller) {
        kturningPID.enableContinuousInput(-180, 180);
        kturningPID.setTolerance(1);
        kStrafingPID.setTolerance(0.5);
        kDrivingPID.setTolerance(0.5);

        /*try {
            if (Limelight.currentPipeline == 0) {
                cone = false;
            } else if (Limelight.currentPipeline == 1) {
                cone = true;
            } 
        } catch (Exception e) {
            // TODO: handle exception
        }*/
        



        this.controller = controller;
        this.m_drive = m_drive;
        addRequirements(m_drive);

        addCommands(
            //alt 2
            //m_drive.gyroSanityCheck(),
            //new RunCommand(() -> m_drive.drive(0, 0, kturningPID.calculate(m_drive.getHeading(), 180), true)).until(() -> kturningPID.atSetpoint()),
            // alterenatively 
            //new RunCommand(() -> m_drive.drive(0, 0, kturningPID.calculate(Limelight.targetRelPos[5], 0), false)).until(() -> kStrafingPID.atSetpoint())
            //new RunCommand(() -> m_drive.drive(0, kStrafingPID.calculate(Limelight.x, 0), kturningPID.calculate(m_drive.getHeading(), 180), false))
            //.until(() -> kStrafingPID.atSetpoint())//,
            //Forward Motion Sequence
            //new RunCommand(() -> m_drive.drive(kDrivingPID.calculate(Limelight.y, yTarget), 0, 0, false)).until(() -> kDrivingPID.atSetpoint())
            new TurnTarget(m_drive),
            new RunCommand(() -> m_drive.drive(translate.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.3)), kStrafingPID.calculate(Limelight.x, 0), kturningPID.calculate(m_drive.getHeading(), 0), false)
            , m_drive)
            //new ForwardCubeTarget(m_drive)
            
        );

    }

}