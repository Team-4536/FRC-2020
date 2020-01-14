package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public interface RobotFrame {
    SpeedController getDrivetrainRightMotor(); 
    SpeedController getDrivetrainLeftMotor();
    SpeedController getShooterFlywheelMotor();
    SpeedController getIntakeMotor();
    SpeedController getBeltMotor();
    //Encoder getDrivetrainRightEncoder();
    //Encoder getDrivetrainLeftEncoder();
    AHRS getDrivetrainNavX();
}

