package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.ISmartMotor;

public class DriveTrain extends SubsystemBase {
    private final ISmartMotor m_leftMotor, m_rightMotor; 
    private final DifferentialDrive m_drive; 
    private final AHRS m_navx;
    private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()));;
    

    public DriveTrain(ISmartMotor leftMotor, ISmartMotor rightMotor, AHRS navx) {
        super();
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
        m_navx = navx;
        m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    }

    @Override
    public void periodic(){
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

    public void reset() {
        m_leftMotor.resetEncoder();
        m_rightMotor.resetEncoder();
    }

    public double getDistance() {
        return (m_leftMotor.getDistance() + m_rightMotor.getDistance())/2;
    }
}