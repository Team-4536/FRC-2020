package frc4536.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.ISmartMotor;

public class DriveTrain extends SubsystemBase {
    private final ISmartMotor m_leftMotor, m_rightMotor; 
    private final DifferentialDrive m_drive; 
    private final AHRS m_navx;
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(0.0));

    public DriveTrain(ISmartMotor leftMotor, ISmartMotor rightMotor, AHRS navx) {
        super();
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
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

    public Pose2d getPose() { //TODO: This only works properly when run in a loop.
        return m_odometry.update(new Rotation2d(getHeading()), m_leftMotor.getSpeed(), m_rightMotor.getSpeed());
    }

    public void closedLoopDrive(double left, double right){ 
        m_leftMotor.setSpeed(left); //TODO: The smartMotors take in angular velocity but we are inputted linear velocities.
        m_rightMotor.setSpeed(right);
    }

    public double getHeading() {
        return m_navx.getYaw(); 
    }

    public void reset() {
        m_leftMotor.resetEncoder();
        m_rightMotor.resetEncoder();
    }
}