package frc4536.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

class SmartMotor implements SpeedController{

    SpeedController motor;
    Encoder encoder;
    PIDController pid;
    public static ArrayList<SmartMotor> motors = new ArrayList<>();

    
    SmartMotor(SpeedController motor, Encoder encoder, double p, double i, double d){
        this.motor = motor;
        this.encoder = encoder;
        pid = new PIDController(p, i, d);
        motors.add(this);

    }


    @Override
    public void pidWrite(double output) {
        // TODO Auto-generated method stub

    }

    @Override
    public void set(double speed) {
        // TODO Auto-generated method stub
        this.motor.set(speed);

    }

    @Override
    public double get() {
        // TODO Auto-generated method stub
        return this.motor.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        // TODO Auto-generated method stub
        this.motor.setInverted(isInverted);

    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        return motor.getInverted();
    }

    @Override
    public void disable() {
        // TODO Auto-generated method stub
        motor.disable();

    }

    @Override
    public void stopMotor() {
        // TODO Auto-generated method stub
        motor.stopMotor();
    }

    public void setSpeed(double speed){
        pid.setSetpoint(speed);
    }

    public void calculate(double setpoint){
        motor.set(pid.calculate(encoder.getRate(), setpoint));
    } 
    public void calculate(){
        calculate(pid.getSetpoint());
    }

}   