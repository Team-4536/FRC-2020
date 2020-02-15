package frc4536.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    // The robot's subsystems and commands are defined here...
    public final RobotFrame m_robotHardware = new TestRobot();
    public final DriveTrain m_driveTrain = new DriveTrain(m_robotHardware.getDrivetrainLeftMotor(),
            m_robotHardware.getDrivetrainRightMotor(),
            m_robotHardware.getDrivetrainNavX(),
            m_robotHardware.getConstants());
    public final Shooter m_shooter = new Shooter(m_robotHardware.getTopShooterFlywheelMotor(), m_robotHardware.getBottomShooterFlywheelMotor());
    public final Conveyor m_conveyor = new Conveyor(m_robotHardware.getBeltMotor(), m_robotHardware.getConveyorBlocker());
    public final Intake m_intake = new Intake(m_robotHardware.getIntakeMotor(), m_robotHardware.getIntakeExtender());
    public final Climber m_climber = new Climber(m_robotHardware.getClimberArmMotor(), m_robotHardware.getLiftMotor());

    private final XboxController m_driveController = new XboxController(0);
    private final Joystick m_liftController = new Joystick(1);

    private final NetworkTableEntry m_xInitial;
    private final NetworkTableEntry m_yInitial;
    
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        //Default behaviour for all subsystems lives here.
        m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.arcadeDrive(-m_driveController.getY(GenericHID.Hand.kLeft), m_driveController.getX(GenericHID.Hand.kRight)), m_driveTrain));

        m_climber.setDefaultCommand(new RunCommand(() -> {
            m_climber.setWinch(m_liftController.getRawButton(7) ? -m_liftController.getY() : 0);
            m_climber.setArm(m_liftController.getRawButton(8) ? -m_liftController.getY() : 0);
        }, m_climber));

        m_conveyor.setDefaultCommand(new RunCommand(() -> {
            m_conveyor.raiseTop();
            m_conveyor.moveConveyor(0);
        }, m_conveyor));

        m_intake.setDefaultCommand(new RunCommand(() -> {
            m_intake.intake(0);
            m_intake.retractIntake();
        }, m_intake));

        m_shooter.setDefaultCommand(new RunCommand(() -> {
            m_shooter.setTopPower(0);
            m_shooter.setBottomPower(0);
        }, m_shooter));

        Shuffleboard.getTab("Subsystems").add(m_climber);
        Shuffleboard.getTab("Subsystems").add(m_conveyor);
        Shuffleboard.getTab("Subsystems").add(m_driveTrain);
        Shuffleboard.getTab("Subsystems").add(m_intake);
        Shuffleboard.getTab("Subsystems").add(m_shooter);

        m_xInitial = Shuffleboard.getTab("Autonomous").add("Initial X", 1.0).getEntry();
        m_yInitial = Shuffleboard.getTab("Autonomous").add("Initial Y", 3.3).getEntry();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driveController, Button.kBumperLeft.value)
                .whileHeld(new PIDCommand(new PIDController(Constants.VISION_KP, Constants.VISION_KI, Constants.VISION_KD),
                        m_driveTrain::getVisionAngle,
                        0,
                        o -> m_driveTrain.arcadeDrive(0, -o),
                        m_driveTrain));

        ShuffleboardTab data = Shuffleboard.getTab("Shooter Data");
        NetworkTableEntry top =  data.add("Top Setpoint",0).getEntry();
        NetworkTableEntry bot = data.add("Bottom Setpoint",0).getEntry();
        new JoystickButton(m_driveController, Button.kBumperRight.value)
                .whileHeld(new IntakeCommands(m_intake, m_conveyor));
        new JoystickButton(m_driveController, Button.kB.value)
                .whenHeld(m_shooter.spinUp(() -> top.getDouble(Constants.SHOOTER_RPS), () -> bot.getDouble(Constants.SHOOTER_RPS)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        m_driveTrain.resetPose(new Pose2d(m_xInitial.getDouble(0.0), m_yInitial.getDouble(0.0), m_driveTrain.getHeading()));
        //TODO: tweak angles

        Pose2d startPosition = new Pose2d(m_xInitial.getDouble(0.0), m_yInitial.getDouble(0.0), Rotation2d.fromDegrees(-180));
        Pose2d shootingPosition = new Pose2d(0.60,5.3, Rotation2d.fromDegrees(-190));
        Pose2d endTrench = new Pose2d(0.69,10.68, Rotation2d.fromDegrees(0));

        // trajectory from initial position to shooting point (end of trench)
        Trajectory initToShoot = TrajectoryGenerator.generateTrajectory(
                List.of(startPosition, shootingPosition),
                m_driveTrain.getConfig()
        );
        // trajectory from shooting position to end of trench
        Trajectory shootToEnd = TrajectoryGenerator.generateTrajectory(
                List.of(shootingPosition, endTrench),
                m_driveTrain.getConfig()
        );
        // return trajectory
        Trajectory endToShoot = TrajectoryGenerator.generateTrajectory(
                List.of(endTrench,shootingPosition),
                m_driveTrain.getConfig().setReversed(true)
        );

        return new SequentialCommandGroup(
                //robot starts in the center of initiation line
                //robot runs shoot command(shooter spin up, put down converyor, run conveyor)
                // wait 2 seconds
                //stop shooter, lift conveyor,
                // put out intake
                //spin up intake
                //scurve to begining of trench
                m_driveTrain.scurveTo(initToShoot),
                //scurve to end of trench
                m_driveTrain.scurveTo(shootToEnd),
                //intake one ball
                // scurve back to begining of trench
                m_driveTrain.scurveTo(endToShoot)
                // run shoot command
        );

        return m_driveTrain.scurveTo(initToShoot).andThen(m_driveTrain.scurveTo(shootToEnd)).andThen(m_driveTrain.scurveTo(endToShoot));
        m_driveTrain.getConfig().setEndVelocity(3));
    }

}
