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
import frc4536.lib.VirtualSmartMotor;

public class TestRobot implements RobotFrame {

    double kP = 10e-5;
    double kI = 1e-6;
    double kD = 0;
    Encoder m_leftEncoder = new Encoder(0,1);
    Encoder m_rightEncoder = new Encoder(2,3);
    PIDController m_PIDLeft = new PIDController(kP, kI, kD);
    PIDController m_PIDRight = new PIDController(kP, kI, kD);
    ISmartMotor m_topFlywheel = new VirtualSmartMotor("Top Flywheel",8.0*0.478779);
    ISmartMotor m_bottomFlywheel = new VirtualSmartMotor("Bottom Flywheel",8.0*0.478779);
    VirtualMotor m_intakeMotor = new VirtualMotor("Intake Motor");
    VirtualMotor m_beltMotor = new VirtualMotor("Belt Motor");
    VirtualMotor m_climberArmMotor = new VirtualMotor("Climber Motor");
    VirtualMotor m_liftMotor = new VirtualMotor("Lift Motor");
    AHRS m_navx = new AHRS();
    SmartMotor m_rightMotors = new SmartMotor(m_rightEncoder, m_PIDRight, new SpeedControllerGroup(new Spark(2), new Spark(3)), 2048);
    SmartMotor m_leftMotors = new SmartMotor(m_leftEncoder, m_PIDLeft, new SpeedControllerGroup(new Spark(0), new Spark(1)), 2048);

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