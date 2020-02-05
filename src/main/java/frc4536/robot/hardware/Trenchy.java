package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import frc4536.lib.*;

public class Trenchy implements RobotFrame {
      // TODO: all of these values are sinful    
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kTrackWidthMeters = 0.53;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public RobotConstants m_constants = new RobotConstants(ksVolts, 
                                                           kvVoltSecondsPerMeter, 
                                                           kaVoltSecondsSquaredPerMeter, 
                                                           kPDriveVel, 
                                                           kTrackWidthMeters, 
                                                           kMaxSpeedMetersPerSecond, 
                                                           kMaxAccelerationMetersPerSecondSquared, 
                                                           kRamseteB, 
                                                           kRamseteZeta); 

    ISmartMotor m_topFlywheel = new VirtualSmartMotor("Top Flywheel",8.0*0.478779); //TODO: REPLACE WITH TALON
    ISmartMotor m_bottomFlywheel = new VirtualSmartMotor("Bottom Flywheel",8.0*0.478779); //TODO: REPLACE WITH TALON
    VirtualMotor m_intakeMotor = new VirtualMotor("Intake Motor");
    VirtualMotor m_beltMotor = new VirtualMotor("Belt Motor");
    Spark m_climberArmMotor = new Spark(7); //TODO: REPLACE WITH VICTOR
    Spark m_liftMotor = new Spark(8); //TODO: REPLACE WITH VICTOR
    AHRS m_navx = new AHRS();
    final PIDConstants kDriveConstants = new PIDConstants(5e-5, 1e-6,0,0);
    NEOSmartMotor m_leftMotors = new NEOSmartMotor(kDriveConstants,1,50,49);
    NEOSmartMotor m_rightMotors = new NEOSmartMotor(kDriveConstants,1,48,47);

    @Override
    public RobotConstants getConstants() {
        return m_constants;
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
    public SpeedController getClimberArmMotor() {
        return m_climberArmMotor;
    }

    @Override
    public SpeedController getLiftMotor() {
        return m_liftMotor;
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
  
    @Override
    public ISmartMotor getTopShooterFlywheelMotor() {
        return m_topFlywheel;
    }

    @Override
    public ISmartMotor getBottomShooterFlywheelMotor() {
        return m_bottomFlywheel;
    }

}
