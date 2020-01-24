package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc4536.lib.ISmartMotor;
import frc4536.lib.SmartMotor;
import frc4536.lib.VirtualMotor;

public class TestRobot implements RobotFrame {
    double kP = 10e-5;
    double kI = 1e-6;
    double kD = 0;
    double kTrackWidthMeters = 24; //TODO: This constant needs to be measured
    Encoder m_leftEncoder = new Encoder(0,1);
    Encoder m_rightEncoder = new Encoder(2,3);
    PIDController m_PIDLeft = new PIDController(kP, kI, kD);
    PIDController m_PIDRight = new PIDController(kP, kI, kD);
    VirtualMotor m_flywheelMotor = new VirtualMotor(4);
    VirtualMotor m_intakeMotor = new VirtualMotor(5);
    VirtualMotor m_beltMotor = new VirtualMotor(6);
    AHRS m_navx = new AHRS();
    SmartMotor m_rightMotors = new SmartMotor(m_rightEncoder, m_PIDRight, new SpeedControllerGroup(new Spark(2), new Spark(3)));
    SmartMotor m_leftMotors = new SmartMotor(m_leftEncoder, m_PIDLeft, new SpeedControllerGroup(new Spark(0), new Spark(1)));

    // TODO: all of these values are sinful    
    public static final class DriveConstants {
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
        // I made this up 
        public static final double kTrackWidthMeters = 0.2;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    @Override
    public ISmartMotor getDrivetrainRightMotor() {
        return m_rightMotors;
    }

    @Override
    public ISmartMotor getDrivetrainLeftMotor() {
        return m_leftMotors;
    }

    @Override
    public SpeedController getShooterFlywheelMotor() {
        return m_flywheelMotor;
    }

    @Override
    public SpeedController getIntakeMotor() {
        return m_intakeMotor;
    }

    @Override
    public SpeedController getBeltMotor() {
        return m_beltMotor;
    }

    @Override
    public AHRS getDrivetrainNavX() {
        return m_navx;
    }
}