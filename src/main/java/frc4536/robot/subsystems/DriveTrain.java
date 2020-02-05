package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.ISmartMotor;

public class DriveTrain extends SubsystemBase {
    private final ISmartMotor m_leftMotor, m_rightMotor; 
    private final DifferentialDrive m_drive; 
    private final AHRS m_navx;
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(0.0));
    double wheelCircumference = 0.0762 * 2 * Math.PI;
    public DriveTrain(ISmartMotor leftMotor, ISmartMotor rightMotor, AHRS navx) {
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
        m_navx = navx;
        m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        ComplexWidget tankDriveTab = Shuffleboard.getTab("Tank Drive Data")
        .add("Tank Drive", m_drive);
    }

    ShuffleboardTab motorBasicTab = Shuffleboard.getTab("Motor Data");

    public void periodic() {
        motorBasicTab.add("Distance Travelled", getDistance());
        motorBasicTab.add("Heading", getHeading());
    }
    
    public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
        m_drive.curvatureDrive(speed, rotation, quickTurn);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
       m_drive.tankDrive(leftSpeed, rightSpeed, false); 
    }

    public Pose2d getPose() { //TODO: This only works properly when run in a loop.
        return m_odometry.update(new Rotation2d(getHeading()), m_leftMotor.getSpeed(), m_rightMotor.getSpeed());
    }

    public void closedLoopDrive(double linLeft, double linRight){ 
        double angScalar = 1;//1/(2 * 0.1524 * Math.PI);
        m_leftMotor.setSpeed(linLeft * angScalar);
        m_rightMotor.setSpeed(linRight * angScalar);
    }

    public double getHeading() {
        return m_navx.getYaw(); 
    }

    public void reset() {
        m_leftMotor.resetEncoder();
        m_rightMotor.resetEncoder();
        m_navx.reset();
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
}
