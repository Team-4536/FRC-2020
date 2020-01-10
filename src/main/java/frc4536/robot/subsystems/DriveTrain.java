
package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    // TODO: implement robot_hardware
    private final SpeedController m_leftMotor = robot_hardware.getDrivetrainLeftMotor();
    private final SpeedController m_rightMotor = robot_hardware.getDrivetrainRightMotor();

    private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    private final Encoder m_leftEncoder = robot_hardware.getDrivetrainLeftEncoder();
    private final Encoder m_rightEncoder = robot_hardware.getDrivetrainRightEncoder();

    private final AHRS navx = robot_hardware.getNavX();

    public DriveTrain() {
        super();
    }
    // TODO: Add shuffleboard logging!

    public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
        m_drive.curvatureDrive(speed, rotation, quickTurn);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
       m_drive.tankDrive(leftSpeed, rightSpeed, false); 
    }

    public double getHeading() {
        return navx.getYaw(); 
    }

    public void reset() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public double getDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance())/2;
    }
}