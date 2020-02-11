package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc4536.lib.IEncoderMotor;
import frc4536.robot.hardware.RobotConstants;

public class DriveTrain extends SubsystemBase {
    private final IEncoderMotor m_leftMotor, m_rightMotor;
    private final AHRS m_navx;
    private Pose2d m_pose;
    private double wheelCircumference = Units.inchesToMeters(3) * 2 * Math.PI;

    public DriveTrain(IEncoderMotor leftMotor, IEncoderMotor rightMotor, AHRS navx, RobotConstants driveConstants) {
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
        m_navx = navx;
        m_driveConstants = driveConstants;
        m_odometry = new DifferentialDriveOdometry(getHeading());
        rightMotor.setInverted(true);

        ShuffleboardTab drivetrain_data = Shuffleboard.getTab("Drivetrain Data");
        drivetrain_data.addNumber("Left Distance", () -> m_leftMotor.getDistance() * wheelCircumference);
        drivetrain_data.addNumber("Right Distance", () -> m_rightMotor.getDistance() * wheelCircumference);
        drivetrain_data.addNumber("Left Velocity", () -> m_leftMotor.getSpeed() * wheelCircumference);
        drivetrain_data.addNumber("Right Velocity", () -> m_rightMotor.getSpeed() * wheelCircumference);
        drivetrain_data.addString("Pose", () -> getPose().toString());
        drivetrain_data.addString("Heading", () -> getHeading().toString());
        drivetrain_data.add("Reset Encoders", new InstantCommand(this::resetEncoders));
        drivetrain_data.add("Reset Pose", new InstantCommand(this::resetPose));
        drivetrain_data.add("Reset Gyro", new InstantCommand(this::resetGyro));
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

    public DifferentialDriveWheelSpeeds getSpeeds(){
        return new DifferentialDriveWheelSpeeds(
                m_leftMotor.getSpeed() * wheelCircumference,
                m_rightMotor.getSpeed() * wheelCircumference
        );
    }

    public Pose2d getPose(){
        return m_pose;
    }

    public void setOutput(double leftVolts, double rightVolts) {
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(rightVolts);
    }

    public void resetEncoders(){
        m_leftMotor.resetEncoder();
        m_rightMotor.resetEncoder();
    }

    public void resetPose(){
        resetEncoders();
        m_odometry.resetPosition(new Pose2d(), getHeading());
    }

    public void resetGyro(){
        m_navx.reset();
        resetPose();
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

