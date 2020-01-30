package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.ISmartMotor;

public class DriveTrain extends SubsystemBase {
    private final ISmartMotor m_leftMotor, m_rightMotor; 
    private final DifferentialDrive m_drive; 
    private final DifferentialDriveOdometry m_odometry;
    private final AHRS m_navx;
    private double maxVelocity;
    private double maxAcceleration;
    private double previousSpeed;

    public DriveTrain(ISmartMotor leftMotor, ISmartMotor rightMotor, AHRS navx) {
        super();
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
        m_navx = navx;
        m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        //DoubleSupplier distance = () -> this.getDistance();
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
    }
    ShuffleboardTab motorBasicTab = Shuffleboard.getTab("Motor Data");

    @Override
    public void periodic() {
        maxAcceleration = Math.max(maxAcceleration, getAcceleration());
        maxVelocity = Math.max(maxVelocity, getVelocity());
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftMotor.getDistance(), m_rightMotor.getDistance());
    }

    public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
        m_drive.curvatureDrive(speed, rotation, quickTurn);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
       m_drive.tankDrive(leftSpeed, rightSpeed, false); 
    }

    public double getHeading() {
        return m_navx.getYaw(); 
    }

    /* (Sasha) I'm keeeping this because it has additional elements, probably will remove it after adding proper updates.
    public Pose2d getPose() { //TODO: This only works properly when run in a loop.
        return m_odometry.update(new Rotation2d(getHeading()), m_leftMotor.getSpeed(), m_rightMotor.getSpeed());
    }
    */

    public void closedLoopDrive(double linLeft, double linRight){ 
        double angScalar = 1/(2 * 0.1524 * Math.PI);
        m_leftMotor.setSpeed(linLeft * angScalar);
        m_rightMotor.setSpeed(linRight * angScalar);
        System.out.println("left: " + m_leftMotor.getSetpoint() + "right: " + m_rightMotor.getSetpoint());
    }

    public void resetEncoders() {
        m_leftMotor.resetEncoder();
        m_rightMotor.resetEncoder();
    }

    public void setVoltages(double left, double right) {
        m_leftMotor.setVolt(left);
        m_rightMotor.setVolt(right);
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
}
