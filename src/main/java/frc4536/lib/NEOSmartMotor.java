package frc4536.lib;

import java.util.ArrayList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class NEOSmartMotor implements ISmartMotor {
private ArrayList<CANSparkMax> motors = new ArrayList<>();
private CANEncoder encoder;
private CANPIDController pid;
private double setpoint;

public NEOSmartMotor(double[] constants, int...deviceID){
    for(int id : deviceID){
        motors.add(new CANSparkMax(id, MotorType.kBrushless));
    }
    encoder = motors.get(0).getEncoder();
    pid = motors.get(0).getPIDController();
    if(constants.length == 1) pid.setP(constants[0]);
    if(constants.length == 2) pid.setI(constants[1]);
    if(constants.length == 3) pid.setD(constants[2]);
    if(constants.length == 4) pid.setIZone(constants[3]);
    if(constants.length == 5) pid.setFF(constants[4]);
}
    @Override
    public void set(double speed) {
        motors.forEach(m -> m.set(speed));
    }   

    @Override
    public double get() {
        return motors.get(0).get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        motors.forEach(m -> m.setInverted(isInverted));
    }

    @Override
    public boolean getInverted() {
        return motors.get(0).getInverted();
    }

    @Override
    public void disable() {
        motors.forEach(m -> m.disable());
    }

    @Override
    public void stopMotor() {
        motors.forEach(m -> m.stopMotor());
    }

    @Override
    public void pidWrite(double output) {
        motors.forEach(m -> m.pidWrite(output));
    }

    @Override
    public void setVolt(double i) {
        motors.forEach(m -> m.setVoltage(i));
    }

    @Override
    public void setSpeed(double i) {
        setpoint = i;
        pid.setReference(setpoint, ControlType.kVelocity);
    }

    @Override
    public double getSpeed() {
        return encoder.getVelocity();
    }

    @Override
    public double getDistance() {
        return encoder.getPosition();
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }


}