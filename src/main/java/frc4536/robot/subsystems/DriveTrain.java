package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private SpeedController m_leftMotor, m_rightMotor; 
    private DifferentialDrive m_drive; 
    private Encoder m_leftEncoder, m_rightEncoder;
    private final AHRS m_navx;

    public DriveTrain(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder, AHRS navx) {
        super();
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
        m_leftEncoder = leftEncoder;
        m_rightEncoder = rightEncoder;
        m_navx = navx;
        m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    }
    // TODO: Add shuffleboard logging!

    public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
        m_drive.curvatureDrive(speed, rotation, quickTurn);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
       m_drive.tankDrive(leftSpeed, rightSpeed, false); 
    }

    public void arcadeDrive(double speed, double rotation) {
        m_drive.arcadeDrive(speed, rotation, false);
    }

    public double getHeading() {
        return m_navx.getYaw(); 
    }

    public void reset() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public double getDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance())/2;
    }
}