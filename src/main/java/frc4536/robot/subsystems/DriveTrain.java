package frc4536.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private SpeedController m_leftMotor, m_rightMotor; 
    private DifferentialDrive m_drive; 
    private Encoder m_leftEncoder, m_rightEncoder;
    private final AHRS m_navx;
    private final DifferentialDriveOdometry m_odometry; 
    
         
    public DriveTrain(final SpeedController leftMotor, final SpeedController rightMotor, final Encoder leftEncoder, final Encoder rightEncoder, final AHRS navx) {
        super();
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
        m_leftEncoder = leftEncoder;
        m_rightEncoder = rightEncoder;
        m_navx = navx;
        m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        m_odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
         
    }
    
    private DifferentialDriveOdometry DifferentialDriveOdometry(Rotation2d fromDegrees) {
        return null;
    }

    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
                          m_rightEncoder.getDistance());
                          // TODO: Add shuffleboard logging!
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }
    
    public void TankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(-rightVolts);
    }
    
    public Encoder getLeftEncoder() {
        return m_leftEncoder;
      }

      public Encoder getRightEncoder() {
        return m_rightEncoder;
      }

      public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
      }
    
    public void curvatureDrive(final double speed, final double rotation, final boolean quickTurn) {
        m_drive.curvatureDrive(speed, rotation, quickTurn);
    }

    public void tankDrive(final double leftSpeed, final double rightSpeed) {
       m_drive.tankDrive(leftSpeed, rightSpeed, false); 
    }

    public double getHeading() {
        return m_navx.getYaw(); 
    }

    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public double getDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance())/2;
    }
}