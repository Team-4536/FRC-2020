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

    IEncoderMotor m_topFlywheel = new VirtualEncoderMotor("Top Flywheel",8.0*0.478779); //TODO: REPLACE WITH TALON
    IEncoderMotor m_bottomFlywheel = new VirtualEncoderMotor("Bottom Flywheel",8.0*0.478779); //TODO: REPLACE WITH TALON
    VirtualMotor m_intakeMotor = new VirtualMotor("Intake Motor");
    VirtualMotor m_beltMotor = new VirtualMotor("Belt Motor");
    Spark m_climberArmMotor = new Spark(7); //TODO: REPLACE WITH VICTOR
    Spark m_liftMotor = new Spark(8); //TODO: REPLACE WITH VICTOR
    AHRS m_navx = new AHRS();
    IEncoderMotor m_leftMotors = new SparkMAX(10.75, 47, 48);
    IEncoderMotor m_rightMotors = new SparkMAX(10.75, 49, 50);

    @Override
    public RobotConstants getConstants() {
        return m_constants;
    }

    @Override
    public IEncoderMotor getDrivetrainRightMotor() {
        return m_rightMotors;
    }

    @Override
    public IEncoderMotor getDrivetrainLeftMotor() {
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
    public IEncoderMotor getTopShooterFlywheelMotor() {
        return m_topFlywheel;
    }

    @Override
    public IEncoderMotor getBottomShooterFlywheelMotor() {
        return m_bottomFlywheel;
    }

}
