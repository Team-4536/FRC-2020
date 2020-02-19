package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import frc4536.lib.IEncoderMotor;

public interface RobotFrame {
    IEncoderMotor getDrivetrainRightMotor();
    IEncoderMotor getDrivetrainLeftMotor();
    IEncoderMotor getTopShooterFlywheelMotor();
    IEncoderMotor getBottomShooterFlywheelMotor();
    SpeedController getIntakeMotor();
    SpeedController getBeltMotor();
    AHRS getDrivetrainNavX();
    RobotConstants getConstants();
    SpeedController getClimberArmMotor();
    SpeedController getLiftMotor();
    DoubleSolenoid getConveyorBlocker();
    DoubleSolenoid getIntakeExtender();
    DigitalInput getBottomLimitSwitch();
}

