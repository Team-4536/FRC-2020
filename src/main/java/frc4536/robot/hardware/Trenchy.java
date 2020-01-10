package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;

public class Trenchy implements RobotFrame {
    private SpeedController m_leftMotor = new SpeedControllerGroup(new Spark(0),
    new Spark(1));
    private SpeedController m_rightMotor = new SpeedControllerGroup(new Spark(2),
    new Spark(3));
    private SpeedController m_flywheelMotor = new Spark(4);
    private SpeedController m_intakeMotor = new Spark(5);
    private SpeedController m_beltMotor = new Spark(6);
    private Encoder m_leftEncoder = new Encoder(0,1);
    private Encoder m_rightEncoder = new Encoder(2,3);

    private AHRS m_navX = new AHRS(SPI.Port.kMXP);

    @Override
    public SpeedController getDrivetrainLeftMotor() {
        return m_leftMotor;
    }
    
    @Override
    public SpeedController getDrivetrainRightMotor() {
        return m_rightMotor;
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
    public Encoder getDrivetrainLeftEncoder() {
        return m_leftEncoder;
    }
    
    @Override
    public Encoder getDrivetrainRightEncoder() {
        return m_rightEncoder;
    }
    
    @Override
    public AHRS getDrivetrainNavX() {
        return m_navX;
    }
}