package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import frc4536.lib.*;

public class Trenchy implements RobotFrame {

    ISmartMotor m_topFlywheel = new VirtualSmartMotor(4,8.0*0.478779); //TODO: REPLACE WITH TALON
    ISmartMotor m_bottomFlywheel = new VirtualSmartMotor(5,8.0*0.478779); //TODO: REPLACE WITH TALON
    VirtualMotor m_intakeMotor = new VirtualMotor(5);
    VirtualMotor m_beltMotor = new VirtualMotor(6);
    Spark m_climberArmMotor = new Spark(7);
    Spark m_liftMotor = new Spark(8);
    AHRS m_navx = new AHRS();
    final PIDConstants kDriveConstants = new PIDConstants(5e-5, 1e-6,0,0);
    NEOSmartMotor m_leftMotors = new NEOSmartMotor(kDriveConstants,1,1,2);
    NEOSmartMotor m_rightMotors = new NEOSmartMotor(kDriveConstants,1,3,4);

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