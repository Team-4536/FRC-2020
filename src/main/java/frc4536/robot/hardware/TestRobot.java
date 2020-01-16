package frc4536.robot.hardware;

import java.util.ArrayList;

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
    double kp = 10e-5;
    double ki = 1e-6;
    double kd = 0;
    Spark leftmotor1 = new Spark(0);
    Spark leftmotor2 = new Spark(1);
    Spark rightmotor1 = new Spark(2);
    Spark rightmotor2 = new Spark(3);
    Encoder leftencoder = new Encoder(0,1);
    Encoder rightencoder = new Encoder(2,3);
    PIDController pidleft = new PIDController(kp, ki, kd);
    PIDController pidright = new PIDController(kp, ki, kd);
    VirtualMotor fakeflywheelmotor = new VirtualMotor(4);
    VirtualMotor fakeintakemotor = new VirtualMotor(5);
    VirtualMotor fakebeltmotor = new VirtualMotor(6);
    AHRS navx = new AHRS();
    SpeedController motors[] = {};
    SmartMotor rightmotors = new SmartMotor(rightencoder, pidright, motors );
    SmartMotor leftmotors = new SmartMotor(leftencoder, pidleft, motors);
    

    @Override
    public ISmartMotor getDrivetrainRightMotor() {
        // TODO Auto-generated method stub
        return rightmotors;
    }

    @Override
    public ISmartMotor getDrivetrainLeftMotor() {
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
    public AHRS getDrivetrainNavX() {
        // TODO Auto-generated method stub
        return navx;
    }

}