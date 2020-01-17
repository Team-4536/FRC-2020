package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import frc4536.lib.ISmartMotor;

public interface RobotFrame {
    ISmartMotor getDrivetrainRightMotor(); 
    ISmartMotor getDrivetrainLeftMotor();
    SpeedController getShooterFlywheelMotor();
    SpeedController getIntakeMotor();
    SpeedController getBeltMotor();
    AHRS getDrivetrainNavX();
}

