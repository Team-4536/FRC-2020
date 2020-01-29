package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc4536.lib.ISmartMotor;
import frc4536.lib.LegacySmartMotor;
import frc4536.lib.PIDConstants;
import frc4536.lib.SmartMotor;
import frc4536.lib.VirtualMotor;

public class TestRobot implements RobotFrame {
    PIDConstants m_pidConstants = new PIDConstants(0.01,0,0);
    Encoder m_leftEncoder = new Encoder(0,1);
    Encoder m_rightEncoder = new Encoder(2,3);
    VirtualMotor m_flywheelMotor = new VirtualMotor(4);
    VirtualMotor m_intakeMotor = new VirtualMotor(5);
    VirtualMotor m_beltMotor = new VirtualMotor(6);
    AHRS m_navx = new AHRS();
    // TODO: these need tick values!
    LegacySmartMotor m_rightMotors = new LegacySmartMotor(m_rightEncoder, 
    m_pidConstants, 
    new SpeedControllerGroup(new Spark(2), new Spark(3)),
    2048,
    8.0);
    LegacySmartMotor m_leftMotors = new LegacySmartMotor(m_leftEncoder, 
    m_pidConstants, 
    new SpeedControllerGroup(new Spark(0), new Spark(1)),
    2048,
    8.0);

    // TODO: all of these values are sinful    
    private final double ksVolts = 2;
    private final double kvVoltSecondsPerMeter = 0.353;
    private final double kaVoltSecondsSquaredPerMeter = 0.00864;
    private final double kPDriveVel = 0.0233;
    private final double kTrackWidthMeters = 0.71;
    private final double kMaxSpeedMetersPerSecond = 8;
    private final double kMaxAccelerationMetersPerSecondSquared = 3;
    private final double kRamseteB = 2;
    private final double kRamseteZeta = 0.7;

    public RobotConstants m_constants = new RobotConstants(ksVolts, 
                                                           kvVoltSecondsPerMeter, 
                                                           kaVoltSecondsSquaredPerMeter, 
                                                           kPDriveVel, 
                                                           kTrackWidthMeters, 
                                                           kMaxSpeedMetersPerSecond, 
                                                           kMaxAccelerationMetersPerSecondSquared, 
                                                           kRamseteB, 
                                                           kRamseteZeta); 

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