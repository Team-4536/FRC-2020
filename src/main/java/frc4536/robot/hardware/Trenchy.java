package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc4536.lib.VirtualMotor;

public class Trenchy implements RobotFrame {
    CANSparkMax leftmotor1 = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax leftmotor2 = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax rightmotor1 = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax rightmotor2 = new CANSparkMax(3, MotorType.kBrushless);
    SpeedControllerGroup leftmotors = new SpeedControllerGroup(leftmotor1, leftmotor2);
    SpeedControllerGroup rightmotors = new SpeedControllerGroup(rightmotor1, rightmotor2);
    VirtualMotor fakeflywheelmotor = new VirtualMotor(4);
    VirtualMotor fakeintakemotor = new VirtualMotor(5);
    VirtualMotor fakebeltmotor = new VirtualMotor(6);
    AHRS navx = new AHRS();

    @Override
    public SpeedController getDrivetrainRightMotor() {
        // TODO Auto-generated method stub
        return rightmotors;
    }

    @Override
    public SpeedController getDrivetrainLeftMotor() {
        // TODO Auto-generated method stub
        return leftmotors;
    }

    @Override
    public SpeedController getShooterFlywheelMotor() {
        // TODO Auto-generated method stub
        return fakeflywheelmotor;
    }

    @Override
    public SpeedController getIntakeMotor() {
        // TODO Auto-generated method stub
        return fakeintakemotor;
    }

    @Override
    public SpeedController getBeltMotor() {
        // TODO Auto-generated method stub
        return fakebeltmotor;
    }

    @Override
    public Encoder getDrivetrainRightEncoder() {
        // TODO Auto-generated method stub
        return rightencoder;
    }

    @Override
    public Encoder getDrivetrainLeftEncoder() {
        // TODO Auto-generated method stub
        return leftencoder;
    }

    @Override
    public AHRS getDrivetrainNavX() {
        // TODO Auto-generated method stub
        return navx;
    }

}