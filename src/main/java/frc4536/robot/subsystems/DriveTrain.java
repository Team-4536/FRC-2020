package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.IEncoderMotor;
import frc4536.robot.hardware.RobotConstants;

import java.util.Arrays;

public class DriveTrain extends SubsystemBase {
    private final IEncoderMotor m_leftMotor, m_rightMotor;
    private final AHRS m_navx;
    private Pose2d m_pose;
    private double maxVelocity, maxAcceleration, previousSpeed;
    private double wheelCircumference = Units.inchesToMeters(3) * 2 * Math.PI;

    public DriveTrain(IEncoderMotor leftMotor, IEncoderMotor rightMotor, AHRS navx, RobotConstants driveConstants) {
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
        m_navx = navx;
        m_driveConstants = driveConstants;
        m_odometry = new DifferentialDriveOdometry(getHeading());
        kinematics = m_driveConstants.kDriveKinematics;
        feedforward = new SimpleMotorFeedforward(m_driveConstants.ksVolts, m_driveConstants.kvVoltSecondsPerMeter, m_driveConstants.kaVoltSecondsSquaredPerMeter);
        leftPID = new PIDController(m_driveConstants.kPDriveVel,0,0);
        rightPID = new PIDController(m_driveConstants.kPDriveVel,0,0);
        rightMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        m_pose = m_odometry.update(getHeading(),
                m_leftMotor.getDistance() * wheelCircumference,
                m_rightMotor.getDistance() * wheelCircumference);
    }

    public void arcadeDrive(double speed, double rotation) {
        double s2 = Math.copySign(speed * speed, speed),
                r2 = Math.copySign(rotation * rotation, rotation);
        m_leftMotor.set(s2 + r2);
        m_rightMotor.set(s2 - r2);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_navx.getAngle());
    }

    private RobotConstants m_driveConstants;
    private DifferentialDriveOdometry m_odometry;
    private DifferentialDriveKinematics kinematics;
    private SimpleMotorFeedforward feedforward;
    private PIDController leftPID;
    private PIDController rightPID;

    public DifferentialDriveWheelSpeeds getSpeeds(){
        return new DifferentialDriveWheelSpeeds(
                m_leftMotor.getSpeed(),
                m_rightMotor.getSpeed()
        );
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public PIDController getLeftPID() {
        return leftPID;
    }

    public PIDController getRightPID() {
        return rightPID;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getPose(){
        return m_pose;
    }

    public void setOutput(double leftVolts, double rightVolts){
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(rightVolts);
    }

    public Command getRamseteAuto(){
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2)); //max sped and accel
        config.setKinematics(getKinematics());
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(), new Pose2d(1,0, new Rotation2d())),
                config);
        RamseteCommand command = new RamseteCommand(trajectory,
                this::getPose,
                new RamseteController(m_driveConstants.kRamseteB, m_driveConstants.kRamseteZeta),
                getFeedforward(),
                getKinematics(),
                this::getSpeeds,
                getLeftPID(),
                getRightPID(),
                this::setOutput,
                this
                );

        return command;
    }


}

//I'm putting some extra code here to clear up the drivetrain clutter. It's only temporary, for me working.
        /*
        init
        ShuffleboardTab motorBasicTab = Shuffleboard.getTab("Motor Data");
        ComplexWidget tankDriveTab = Shuffleboard.getTab("Tank Drive Data")
        .add("Tank Drive", m_drive);
        motorBasicTab.addNumber("Distance Travelled", () -> getDistance());
        motorBasicTab.addNumber("Heading", () -> getHeading());
        motorBasicTab.addNumber("Velocity", () -> getVelocity());
        motorBasicTab.addNumber("Max Velocity", () -> getMaxVelocity());
        motorBasicTab.addNumber("Acceleration", () -> getAcceleration());
        motorBasicTab.addNumber("Max Acceleration", () -> getMaxAcceleration());
        maxVelocity = 0;
        maxAcceleration = 0;
        previousSpeed = 0;

        periodic
        maxAcceleration = Math.max(maxAcceleration, getAcceleration());
        maxVelocity = Math.max(maxVelocity, getVelocity());

         (Sasha) I'm keeeping this because it has additional elements, probably will remove it after adding proper updates.
    public Pose2d getPose() { //TODO: This only works properly when run in a loop.
        return m_odometry.update(new Rotation2d(getHeading()), m_leftMotor.getSpeed(), m_rightMotor.getSpeed());
    }


    public void reset() {
        m_leftMotor.resetEncoder();
        m_rightMotor.resetEncoder();
        m_navx.reset();
    }

        public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
        m_drive.curvatureDrive(speed, rotation, quickTurn);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed, false);
    }


    public double getLeftSpeed() {
        return m_leftMotor.getSpeed() * wheelCircumference;
    }
    public double getRightSpeed() {
        return m_rightMotor.getSpeed() * wheelCircumference;
    }

    public double getLeftDistance() {
        return m_leftMotor.getDistance() * wheelCircumference;
    }
    public double getRightDistance() {
        return m_rightMotor.getDistance() * wheelCircumference;
    }

    public void setVoltages(double left, double right) {
        m_leftMotor.setVoltage(left);
        m_rightMotor.setVoltage(right);
        m_drive.feed();
        //TODO: May need to feed the watchdog here, Oblarg added that to the WPILIb example
    }

    public double getVelocity() {
        return (m_leftMotor.getSpeed() + m_rightMotor.getSpeed())/2;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getAcceleration() {
        return Math.hypot(m_navx.getWorldLinearAccelX() * 9.8, m_navx.getWorldLinearAccelY() * 9.8); //Acceleration is in Gs by the NavX, convert to m/s^2
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getDistance() {
        return (m_leftMotor.getDistance() + m_rightMotor.getDistance())/2;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_rightMotor.getSpeed(), m_leftMotor.getSpeed());
    }

         */

