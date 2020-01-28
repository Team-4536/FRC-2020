package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc4536.lib.ISmartMotor;
import frc4536.lib.SmartMotor;
import frc4536.lib.VirtualMotor;

public class TestRobot implements RobotFrame {
    double kP = 10e-5;
    double kI = 1e-6;
    double kD = 0;
    Encoder m_leftEncoder = new Encoder(0,1);
    Encoder m_rightEncoder = new Encoder(2,3);
    PIDController m_PIDLeft = new PIDController(kP, kI, kD);
    PIDController m_PIDRight = new PIDController(kP, kI, kD);
    VirtualMotor m_flywheelMotor = new VirtualMotor(4);
    VirtualMotor m_intakeMotor = new VirtualMotor(5);
    VirtualMotor m_beltMotor = new VirtualMotor(6);
    AHRS m_navx = new AHRS();
    // TODO: these need tick values!
    SmartMotor m_rightMotors = new SmartMotor(m_rightEncoder, m_PIDRight, new SpeedControllerGroup(new Spark(2), new Spark(3)));
    SmartMotor m_leftMotors = new SmartMotor(m_leftEncoder, m_PIDLeft, new SpeedControllerGroup(new Spark(0), new Spark(1)));


    // TODO: all of these values are sinful    
    private final double ksVolts = 0.22;
    private final double kvVoltSecondsPerMeter = 1.98;
    private final double kaVoltSecondsSquaredPerMeter = 0.2;
    private final double kPDriveVel = 8.5;
    private final double kTrackWidthMeters = 0.71;
    private final double kMaxSpeedMetersPerSecond = 3;
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