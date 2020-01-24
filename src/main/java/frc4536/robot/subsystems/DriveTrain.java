package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.ISmartMotor;

public class DriveTrain extends SubsystemBase {
    private final ISmartMotor m_leftMotor, m_rightMotor; 
    private final DifferentialDrive m_drive; 
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
        ComplexWidget tankDriveTab = Shuffleboard.getTab("Tank Drive Data")
        .add("Tank Drive", m_drive);
        maxVelocity = 0;
        maxAcceleration = 0;
        previousSpeed = 0;
    }
    ShuffleboardTab motorBasicTab = Shuffleboard.getTab("Motor Data");

    public void periodic() {
        motorBasicTab.add("Distance Travelled", getDistance());
        motorBasicTab.add("Heading", getHeading());
        motorBasicTab.add("Velocity", getVelocity());
        motorBasicTab.add("Max Velocity", Math.max(maxVelocity, getVelocity()));
        maxVelocity = Math.max(maxVelocity, getVelocity());
        motorBasicTab.add("Acceleleration", Math.max(maxAcceleration, getAcceleration()));
        maxAcceleration = Math.max(maxAcceleration, getAcceleration());
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

    public void reset() {
        m_leftMotor.resetEncoder();
        m_rightMotor.resetEncoder();
    }

    public double getVelocity() {
        return (m_leftMotor.getSpeed() + m_rightMotor.getSpeed())/2;
    }

    public double getAcceleration() {
        return (getVelocity()-previousSpeed)*50;
    }

    public double getDistance() {
        return (m_leftMotor.getDistance() + m_rightMotor.getDistance())/2;
    }
}