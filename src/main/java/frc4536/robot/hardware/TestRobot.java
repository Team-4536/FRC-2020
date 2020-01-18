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
    Encoder leftencoder = new Encoder(0,1);
    Encoder rightencoder = new Encoder(2,3);
    PIDController pidleft = new PIDController(kp, ki, kd);
    PIDController pidright = new PIDController(kp, ki, kd);
    VirtualMotor m_flywheelMotor = new VirtualMotor(4);
    VirtualMotor m_intakeMotor = new VirtualMotor(5);
    VirtualMotor fakebeltmotor = new VirtualMotor(6);
    AHRS navx = new AHRS();
    SmartMotor m_rightMotors = new SmartMotor(rightencoder, pidright, new SpeedControllerGroup(new Spark(2), new Spark(3)));
    SmartMotor m_leftMotors = new SmartMotor(leftencoder, pidleft, new SpeedControllerGroup(new Spark(0), new Spark(1)));
    

    @Override
    public ISmartMotor getDrivetrainRightMotor() {
        // TODO Auto-generated method stub
        return m_rightMotors;
    }

    @Override
    public ISmartMotor getDrivetrainLeftMotor() {
        // TODO Auto-generated method stub
        return m_leftMotors;
    }

    @Override
    public SpeedController getShooterFlywheelMotor() {
        // TODO Auto-generated method stub
        return m_flywheelMotor;
    }

    @Override
    public SpeedController getIntakeMotor() {
        // TODO Auto-generated method stub
        return m_intakeMotor;
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