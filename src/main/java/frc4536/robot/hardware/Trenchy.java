package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc4536.lib.ISmartMotor;
import frc4536.lib.NEOSmartMotor;
import frc4536.lib.PIDConstants;
import frc4536.lib.VirtualMotor;

public class Trenchy implements RobotFrame {

    VirtualMotor m_flywheelMotor = new VirtualMotor(4);
    VirtualMotor m_intakeMotor = new VirtualMotor(5);
    VirtualMotor m_beltMotor = new VirtualMotor(6);
    AHRS m_navx = new AHRS();
    final PIDConstants kDriveConstants = new PIDConstants(5e-5, 1e-6,0,0);
    // TODO: tweak tick constant
    NEOSmartMotor m_leftMotors = new NEOSmartMotor(kDriveConstants,1,1,2);
    NEOSmartMotor m_rightMotors = new NEOSmartMotor(kDriveConstants,1,3,4);


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
        public static final double kTrackWidthMeters = 0.53;
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
