package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc4536.lib.VirtualMotor;

public class TestRobot implements RobotFrame {
    Spark leftmotor1 = new Spark(0);
    Spark leftmotor2 = new Spark(1);
    Spark rightmotor1 = new Spark(2);
    Spark rightmotor2 = new Spark(3);
    SpeedControllerGroup leftmotors = new SpeedControllerGroup(leftmotor1, leftmotor2);
    SpeedControllerGroup rightmotors = new SpeedControllerGroup(rightmotor1, rightmotor2);
    VirtualMotor fakeflywheelmotor = new VirtualMotor(4);
    VirtualMotor fakeintakemotor = new VirtualMotor(5);
    VirtualMotor fakebeltmotor = new VirtualMotor(6);
    Encoder leftencoder = new Encoder(0, 1);
    Encoder rightencoder = new Encoder(2, 3);
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