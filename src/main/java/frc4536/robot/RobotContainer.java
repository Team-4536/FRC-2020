package frc4536.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4536.robot.commands.autos.*;
import frc4536.robot.commands.*;
import frc4536.robot.hardware.*;
import frc4536.robot.subsystems.*;

import java.util.ArrayList;

import static frc4536.lib.Utilities.deadzone;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here.
    public final RobotFrame m_robotHardware = new Honeycomb();
    public final DriveTrain m_driveTrain = new DriveTrain(m_robotHardware.getDrivetrainLeftMotor(),
            m_robotHardware.getDrivetrainRightMotor(),
            m_robotHardware.getDrivetrainNavX(),
            m_robotHardware.getConstants());
    public final Shooter m_shooter = new Shooter(m_robotHardware.getTopShooterFlywheelMotor(), m_robotHardware.getBottomShooterFlywheelMotor());
    public final Conveyor m_conveyor = new Conveyor(m_robotHardware.getBeltMotor(), m_robotHardware.getConveyorBlocker(), m_robotHardware.getConveyorBeam());
    public final Intake m_intake = new Intake(m_robotHardware.getIntakeMotor(), m_robotHardware.getIntakeExtender());
    public final Climber m_climber = new Climber(m_robotHardware.getClimberArmMotor(), m_robotHardware.getLiftMotor(), m_robotHardware.getBottomLimitSwitch());

    private final XboxController m_driveController = new XboxController(0);
    private final Joystick m_operatorJoystick = new Joystick(1);

    private final NetworkTableEntry m_xInitial, m_yInitial;
    private final SendableChooser<Autonomous> m_chooser = new SendableChooser<>();

    Trajectory t_startToShoot = TrajectoryGenerator.generateTrajectory(Poses.TRENCH_START,
            new ArrayList<>(),
            Poses.AUTO_TRENCH_SHOOT,
            m_driveTrain.getConfig().setReversed(true)); //done
    Trajectory t_shootToEnd = TrajectoryGenerator.generateTrajectory(Poses.AUTO_TRENCH_SHOOT,
            new ArrayList<>(),
            Poses.TRENCH_END,
            m_driveTrain.getConfig().setReversed(true)); //done
    Trajectory t_endToShoot = TrajectoryGenerator.generateTrajectory(Poses.TRENCH_END,
            new ArrayList<>(),
            Poses.AUTO_TRENCH_SHOOT,
            m_driveTrain.getConfig().setReversed(false)); //done

    Trajectory t_toRendezShoot = TrajectoryGenerator.generateTrajectory(Poses.TRENCH_START,
            new ArrayList<>(),
            Poses.RENDEZ_SHOOT,
            m_driveTrain.getConfig().setReversed(true)); //done
    Trajectory t_shootToRendez = TrajectoryGenerator.generateTrajectory(Poses.RENDEZ_SHOOT,
            new ArrayList<>(),
            Poses.RENDEZ_SWEEP,
            m_driveTrain.getConfig().setReversed(true)); //done
    Trajectory t_rendezToShoot = TrajectoryGenerator.generateTrajectory(Poses.RENDEZ_SWEEP,
            new ArrayList<>(),
            Poses.RENDEZ_SHOOT,
            m_driveTrain.getConfig().setReversed(false)); //done
    Trajectory t_centerAuto = TrajectoryGenerator.generateTrajectory(Poses.CENTER_AUTO_START,
            new ArrayList<>(),
            Poses.CENTER_AUTO_END,
            m_driveTrain.getConfig().setReversed(true)); //done


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();

        ShuffleboardTab auto = Shuffleboard.getTab("Autonomous");


        m_xInitial = auto.add("Initial X", 3.1).getEntry();
        m_yInitial = auto.add("Initial Y", -2.45).getEntry();
        m_chooser.addOption("Physical Diagnostic", Autonomous.PHYSICAL_DIAGNOSTIC);
        m_chooser.addOption("Trench", Autonomous.TRENCH);
        m_chooser.addOption("Vision Test", Autonomous.VISION_TEST);
        m_chooser.addOption("Rendezvous", Autonomous.RENDEZVOUS);
        m_chooser.setDefaultOption("Center Auto", Autonomous.CENTER_AUTO);
        m_chooser.addOption("Baseline", Autonomous.BASELINE);
        m_chooser.addOption("Opposite Trench", Autonomous.TRENCH_STEAL);
        auto.add(m_chooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //Drive Controller
        new JoystickButton(m_driveController, Button.kBumperLeft.value)
                .whileHeld(new VisionToTargetCommand(m_driveTrain));    //vision
        new JoystickButton(m_driveController, Button.kBumperRight.value)
                .whileHeld(new IntakeCommands(m_intake, m_conveyor, m_driveController::getAButton));   //Intake
        new JoystickButton(m_driveController, Button.kB.value)          //Initiate Shooting
                .whileHeld(() -> {
                    m_conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED, true);
                    m_conveyor.lowerTop();
                }, m_conveyor);
        new JoystickButton(m_driveController, Button.kY.value)          //Spin up shooter and automatically fires when shooter reaches a speed.
                .whileHeld(new ShootCommand(m_shooter, m_conveyor));
        new JoystickButton(m_driveController, Button.kX.value)          //reverse move conveyor 
                .whileHeld(new RunCommand(() -> m_conveyor.moveConveyor(-Constants.CONVEYOR_INTAKE_SPEED, true), m_conveyor));

        //Operator Controller
        new JoystickButton(m_operatorJoystick, 12) //Intake extend
                .whileHeld(new RunCommand(m_intake::extendIntake, m_intake));
        new JoystickButton(m_operatorJoystick, 11) //Conveyor lower
                .whileHeld(new RunCommand(m_conveyor::lowerTop, m_conveyor));
        new JoystickButton(m_operatorJoystick, 7) //Conveyor manual control
                .whileHeld(new RunCommand(() -> m_conveyor.moveConveyor(-m_operatorJoystick.getY(), true), m_conveyor));
        new JoystickButton(m_operatorJoystick, 8)   //Intake manual control
                .whileHeld(new RunCommand(() -> m_intake.intake(-m_operatorJoystick.getY()), m_intake));
        new JoystickButton(m_operatorJoystick, 2) //Spinup shooter
                .whileHeld(new ManualShooter(m_operatorJoystick::getX, m_driveTrain, m_shooter));
        new JoystickButton(m_operatorJoystick, 1) //manual shoot
                .whileHeld(new RunCommand(() -> {
                    m_conveyor.lowerTop();
                    m_conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED, true);
                }, m_conveyor));
    }

    private void configureDefaultCommands() {
        //Default behaviour for all subsystems lives here.
        CommandBase default_driveTrain = new RunCommand(() -> {
            boolean trigger = m_driveController.getTriggerAxis(Hand.kLeft) > 0.5;
            boolean button = m_operatorJoystick.getRawButton(5);
            m_driveTrain.arcadeDrive(
                    (button ? 0.4 : (trigger ? 0.6 : 1.0)) * deadzone(m_driveController.getY(GenericHID.Hand.kLeft), Constants.DRIVE_DEADZONE),
                    (button ? 0.4 : (trigger ? 0.6 : 0.9)) * deadzone(m_driveController.getX(GenericHID.Hand.kRight), Constants.DRIVE_DEADZONE),
                    true);
        }, m_driveTrain);
        CommandBase default_climber = new RunCommand(() -> {  //climber
            m_climber.setWinch(m_operatorJoystick.getRawButton(3) ? -1 :(m_operatorJoystick.getRawButton(4) ? -m_operatorJoystick.getY() : 0));
            m_climber.setArm(m_operatorJoystick.getRawButton(5) ? -m_operatorJoystick.getY() : 0);
        }, m_climber);
        CommandBase default_conveyor = new RunCommand(() -> { //conveyor
            m_conveyor.raiseTop();
            m_conveyor.moveConveyor(0, true);
        }, m_conveyor);
        CommandBase default_intake = new RunCommand(() -> {   //intake
            m_intake.intake(0);
            m_intake.retractIntake();
        }, m_intake);
        CommandBase default_shooter = new RunCommand(() -> {  //shooter
            if (m_driveController.getTriggerAxis(Hand.kRight) > 0.5) { //change to 0.5
                m_shooter.setSetpoints();
            } else {
                m_shooter.setTopPower(0);
                m_shooter.setBottomPower(0);
            }
        }, m_shooter);

        default_climber.setName("Default Climber");
        default_conveyor.setName("Default Conveyor");
        default_shooter.setName("Default Shooter");
        default_intake.setName("Default Intake");
        default_driveTrain.setName("Default Drivetrain");

        m_driveTrain.setDefaultCommand(default_driveTrain);
        m_climber.setDefaultCommand(default_climber);
        m_conveyor.setDefaultCommand(default_conveyor);
        m_intake.setDefaultCommand(default_intake);
        m_shooter.setDefaultCommand(default_shooter);
    }

    public Command generateAutoCommands(Autonomous chose, Pose2d initialPose) {
        switch (chose) {
            case PHYSICAL_DIAGNOSTIC:
                return new PhysicalDiagnostic(m_shooter, m_conveyor, m_intake);
            case TRENCH:
                return new TrenchAutoCommand(m_shooter, m_conveyor, m_driveTrain, m_intake, initialPose, t_shootToEnd, t_endToShoot);
            case VISION_TEST:
                return new VisionTestAutoCommand(m_shooter, m_conveyor, m_driveTrain, m_intake, t_startToShoot);
            case RENDEZVOUS:
                return new RendezvousAutoCommand(m_shooter, m_conveyor, m_driveTrain, m_intake, initialPose, t_shootToRendez, t_rendezToShoot);
            case CENTER_AUTO:
                return new CenterAutoCommand(m_shooter, m_conveyor, m_driveTrain, initialPose);
            case BASELINE:
                return new RunCommand(() -> m_driveTrain.arcadeDrive(-0.3, 0), m_driveTrain).withTimeout(1).andThen(new RunCommand(() -> m_driveTrain.arcadeDrive(0, 0), m_driveTrain));
            default:
                return new CenterAutoCommand(m_shooter, m_conveyor, m_driveTrain, initialPose); //safety purposes lol
        }
        //m_chooser.addOption("Test Auto", m_testAuto);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        m_driveTrain.resetGyro();
        Pose2d initialPose = new Pose2d(m_xInitial.getDouble(0.0), m_yInitial.getDouble(0.0), Rotation2d.fromDegrees(0.0));
        m_driveTrain.resetPose(initialPose);
        return generateAutoCommands(m_chooser.getSelected(), initialPose);
    }

    private enum Autonomous {
        PHYSICAL_DIAGNOSTIC,
        TRENCH,
        VISION_TEST,
        RENDEZVOUS,
        BASELINE,
        CENTER_AUTO,
        TRENCH_STEAL;
    }
}
