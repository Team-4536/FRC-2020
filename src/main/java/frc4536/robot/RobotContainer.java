package frc4536.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4536.robot.commands.*;
import frc4536.robot.hardware.*;
import frc4536.robot.subsystems.*;

import java.util.List;

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
    public final Conveyor m_conveyor = new Conveyor(m_robotHardware.getBeltMotor(), m_robotHardware.getConveyorBlocker());
    public final Intake m_intake = new Intake(m_robotHardware.getIntakeMotor(), m_robotHardware.getIntakeExtender());
    public final Climber m_climber = new Climber(m_robotHardware.getClimberArmMotor(), m_robotHardware.getLiftMotor(), m_robotHardware.getBottomLimitSwitch());

    private final XboxController m_driveController = new XboxController(0);
    private final Joystick m_operatorJoystick = new Joystick(1);

    private final NetworkTableEntry m_xInitial, m_yInitial, top, bot;
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();

        ShuffleboardTab data = Shuffleboard.getTab("Shooter Data"),
                subsystems = Shuffleboard.getTab("Subsystems"),
                auto = Shuffleboard.getTab("Autonomous");

        subsystems.add(m_climber);
        subsystems.add(m_conveyor);
        subsystems.add(m_driveTrain);
        subsystems.add(m_intake);
        subsystems.add(m_shooter);

        top = data.add("Top Setpoint", Constants.SHOOTER_RPS_TOP).getEntry();
        bot = data.add("Bottom Setpoint", Constants.SHOOTER_RPS_BOTTOM).getEntry();

        m_xInitial = auto.add("Initial X", 3.3).getEntry();
        m_yInitial = auto.add("Initial Y", -1.0).getEntry();
        generateAutoCommands();
        auto.add(m_chooser);

        subsystems.add("Conv L", new RunCommand(() -> m_conveyor.lowerTop(), m_conveyor).withTimeout(3));
        subsystems.add("Conv R", new RunCommand(() -> m_conveyor.raiseTop(), m_conveyor).withTimeout(3));
        subsystems.add("Int Retract", new RunCommand(() -> m_intake.retractIntake(), m_intake).withTimeout(3));
        subsystems.add("Int Extemd", new RunCommand(() -> m_intake.extendIntake(), m_intake).withTimeout(3));
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
                .whileHeld(new IntakeCommands(m_intake, m_conveyor));   //Intake
        new JoystickButton(m_driveController, Button.kB.value)          //Spin up conveyor
                .whileHeld(() -> m_conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED), m_conveyor);
        new JoystickButton(m_driveController, Button.kY.value)          //Spin up shooter
                .whileHeld(new ShootCommand(m_shooter, m_conveyor, () -> top.getDouble(Constants.SHOOTER_RPS_TOP), () -> bot.getDouble(Constants.SHOOTER_RPS_BOTTOM)));

        //Operator Controller
        new JoystickButton(m_operatorJoystick, 2)   //Spinup shooter
                .whileHeld(m_shooter.spinUp(() -> top.getDouble(Constants.SHOOTER_RPS_TOP), () -> bot.getDouble(Constants.SHOOTER_RPS_BOTTOM)));
        new JoystickButton(m_operatorJoystick, 7)   //Conveyor manual control
                .whileHeld(new RunCommand(() -> m_conveyor.moveConveyor(-m_operatorJoystick.getY()), m_conveyor));
        new JoystickButton(m_operatorJoystick, 8)   //Intake manual control
                .whileHeld(new RunCommand(() -> m_intake.intake(-m_operatorJoystick.getY()), m_intake));
        new JoystickButton(m_operatorJoystick, 11)  //Conveyor lower
                .whileHeld(new RunCommand(() -> m_conveyor.lowerTop(), m_conveyor));
        new JoystickButton(m_operatorJoystick, 12)  //Intake extend
                .whileHeld(new RunCommand(() -> m_intake.extendIntake(), m_intake));
    }

    private void configureDefaultCommands() {
        //Default behaviour for all subsystems lives here.
        CommandBase default_driveTrain = new RunCommand(() -> m_driveTrain.arcadeDrive( //driver train
            Math.abs(m_driveController.getY(GenericHID.Hand.kLeft)) > Constants.DRIVE_DEADZONE ? -m_driveController.getY(GenericHID.Hand.kLeft) : 0,
            Math.abs(m_driveController.getX(GenericHID.Hand.kRight)) > Constants.DRIVE_DEADZONE ? m_driveController.getX(GenericHID.Hand.kRight) : 0),
            m_driveTrain);
        CommandBase default_climber = new RunCommand(() -> {  //climber
                if(m_operatorJoystick.getRawButton(9)) m_climber.setWinch(-m_operatorJoystick.getY());
                if(m_operatorJoystick.getRawButton(10)) m_climber.setArm(-m_operatorJoystick.getY());
                else{
            m_climber.setWinch(0);
            m_climber.setArm(0);
                }
        }, m_climber);
        CommandBase default_conveyor = new RunCommand(() -> { //conveyor
            m_conveyor.raiseTop();
            m_conveyor.moveConveyor(0);
        }, m_conveyor);
        CommandBase default_intake = new RunCommand(() -> {   //intake
            m_intake.intake(0);
            m_intake.retractIntake();
        }, m_intake);
        CommandBase default_shooter = new RunCommand(() -> {  //shooter
            if (m_driveController.getTriggerAxis(Hand.kRight) > 0.5) { //change to 0.5
                m_shooter.spinUp(() -> top.getDouble(Constants.SHOOTER_RPS_TOP), () -> bot.getDouble(Constants.SHOOTER_RPS_BOTTOM)).schedule();
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


    public void generateAutoCommands() {
        Pose2d startPosition = new Pose2d(m_xInitial.getDouble(0.0), m_yInitial.getDouble(0.0), Rotation2d.fromDegrees(0));
        Pose2d shootingPosition = new Pose2d(7.3, -0.465, Rotation2d.fromDegrees(12));
        Pose2d endTrench = new Pose2d(7.341, -0.465, Rotation2d.fromDegrees(0));
        Pose2d twoBallPosition = new Pose2d(6.312, -2.903, Rotation2d.fromDegrees(-90));
        Pose2d testPosition = new Pose2d(4,-3, Rotation2d.fromDegrees(0));
        // trajectory from shooting position to end of trench
        Trajectory initToEnd = TrajectoryGenerator.generateTrajectory(
                startPosition,
                List.of(new Translation2d(5.106, -0.465)),
                endTrench,
                m_driveTrain.getConfig().setReversed(false)
        );
        Trajectory shotToEnd = TrajectoryGenerator.generateTrajectory(
                shootingPosition,
                List.of(new Translation2d(5.106, -0.465)),
                endTrench,
                m_driveTrain.getConfig().setReversed(false)
        );
        // return trajectory
        Trajectory endToShoot = TrajectoryGenerator.generateTrajectory(
                endTrench,
                List.of(new Translation2d(5.407, -1.758)),
                shootingPosition,
                m_driveTrain.getConfig().setReversed(true)
        );

        Trajectory shootTo2Ball = TrajectoryGenerator.generateTrajectory(
                shootingPosition,
                List.of(new Translation2d(5.106, -1.7)),
                twoBallPosition,
                m_driveTrain.getConfig().setReversed(false)
        );

        Trajectory shootAgain = TrajectoryGenerator.generateTrajectory(
                twoBallPosition,
                List.of(new Translation2d(5.106, -1.7)),
                shootingPosition,
                m_driveTrain.getConfig().setReversed(true)
        );

        final Command m_trenchAuto = new SequentialCommandGroup(
                m_driveTrain.scurveTo(initToEnd).raceWith(new IntakeCommands(m_intake, m_conveyor)), //scurve to balls with the intake out
                m_driveTrain.scurveTo(endToShoot).raceWith(m_shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)), //scurve to the shooting position and preemptively spin up the shooter
                new VisionToTargetCommand(m_driveTrain).raceWith(m_shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)),
                new ShootCommand(m_shooter, m_conveyor, 0.0),

                //time to attempt to pick up more balls.
                m_driveTrain.scurveTo(shootTo2Ball).raceWith(new IntakeCommands(m_intake, m_conveyor)), //scurve to balls with the intake out
                m_driveTrain.scurveTo(shootAgain).raceWith(m_shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)), //scurve to the shooting position and preemptively spin up the shooter
                new VisionToTargetCommand(m_driveTrain).raceWith(m_shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)),
                new ShootCommand(m_shooter, m_conveyor, 0.0)
        );

        final Command m_eightBallAuto = new SequentialCommandGroup(
                m_driveTrain.scurveTo(TrajectoryGenerator.generateTrajectory(startPosition, List.of(new Translation2d(5.106, -0.465)), shootingPosition, m_driveTrain.getConfig().setReversed(false))).raceWith(new IntakeCommands(m_intake, m_conveyor), m_shooter.spinUp()),
                new ShootCommand(m_shooter, m_conveyor),
                m_driveTrain.scurveTo(shotToEnd).raceWith(new IntakeCommands(m_intake, m_conveyor)),
                m_driveTrain.scurveTo(endToShoot).raceWith(m_shooter.spinUp()),
                new ShootCommand(m_shooter, m_conveyor)

        );

        final Command m_testAuto = new SequentialCommandGroup( m_driveTrain.scurveTo(
                TrajectoryGenerator.generateTrajectory(
                        startPosition, 
                        List.of(new Translation2d(
                                m_xInitial.getDouble(0.0)+3,0)), testPosition, m_driveTrain.getConfig().setReversed(true))));

        m_chooser.setDefaultOption("Trench Auto", m_trenchAuto);
        m_chooser.addOption("Eight Ball Auto", m_eightBallAuto);
        m_chooser.addOption("Test Auto", m_testAuto);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        m_driveTrain.resetPose(new Pose2d(m_xInitial.getDouble(0.0), m_yInitial.getDouble(0.0), m_driveTrain.getHeading()));
        //TODO: tweak angles
        return m_chooser.getSelected();


        //Same thing as above, but worse
        /*
                    m_driveTrain.scurveTo(initToEnd)
                    .raceWith(new IntakeCommands(m_intake, m_conveyor))
                    .andThen(m_driveTrain.scurveTo(endToShoot)
                            .raceWith(m_shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)))
                    .andThen(new RunCommand(m_conveyor::lowerTop).alongWith(new RunCommand(() -> m_conveyor.moveConveyor(Constants.CONVEYOR_SPEED))))
                    .andThen(m_driveTrain.scurveTo(shootTo2Ball)
                            .raceWith(new IntakeCommands(m_intake, m_conveyor)))
                    .andThen(m_driveTrain.scurveTo(shootAgain).raceWith(m_shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)))
                    .andThen(new RunCommand(m_conveyor::lowerTop).alongWith(new RunCommand(() -> m_conveyor.moveConveyor(Constants.CONVEYOR_SPEED))));


       //robot starts in the center of initiation line
        //robot runs shoot command(shooter spin up, put down converyor, run conveyor)
        //stop shooter, lift conveyor,
        // put out intake
        //spin up intake
        //scurve from init to end of trench
        //intake one ball
        // scurve back to begining of trench
        // run shoot command

        return new ParallelCommandGroup(
                //new RunCommand(m_intake::extendIntake).withTimeout(1),
                //new RunCommand(() -> m_intake.intake(1)),
                new SequentialCommandGroup(
                        //new ShootCommand(m_conveyor, m_shooter, () -> 70, () -> 70)
                        //       .withTimeout(5)
                        //       .andThen(m_conveyor::raiseTop),
                        m_driveTrain.scurveTo(initToEnd),
                        m_driveTrain.scurveTo(endToShoot),
                        new ShootCommand(m_conveyor, m_shooter, () -> 70, () -> 70)
                                .withTimeout(5)
                                .andThen(m_conveyor::raiseTop),
                        m_driveTrain.scurveTo(shootTo2Ball),
                        m_driveTrain.scurveTo(shootAgain),
                        new ShootCommand(m_conveyor, m_shooter, () -> 70, () -> 70)
                                .withTimeout(5)
                                .andThen(m_conveyor::raiseTop)
                )
        );

             */
    }
}
