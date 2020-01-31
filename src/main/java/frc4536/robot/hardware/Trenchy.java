package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import frc4536.lib.ISmartMotor;
import frc4536.lib.NEOSmartMotor;
import frc4536.lib.PIDConstants;
import frc4536.lib.VirtualMotor;

public class Trenchy implements RobotFrame {

    VirtualMotor m_flywheelMotor = new VirtualMotor(4);
    VirtualMotor m_intakeMotor = new VirtualMotor(5);
    VirtualMotor m_beltMotor = new VirtualMotor(6);
    Spark m_climberArmMotor = new Spark(7);
    Spark m_liftMotor = new Spark(8);
    AHRS m_navx = new AHRS();
    final PIDConstants kDriveConstants = new PIDConstants(5e-5, 1e-6,0,0);
    NEOSmartMotor m_leftMotors = new NEOSmartMotor(kDriveConstants,1,0,1);
    NEOSmartMotor m_rightMotors = new NEOSmartMotor(kDriveConstants,1,2,3);

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