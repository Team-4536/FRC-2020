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

public NEOSmartMotor(PIDConstants ks, int...deviceID){
    for(int id : deviceID){
        motors.add(new CANSparkMax(id, MotorType.kBrushless));
    }
    encoder = motors.get(0).getEncoder();
    pid = motors.get(0).getPIDController();

    pid.setP(ks.kP);
    pid.setI(ks.kI);
    pid.setD(ks.kD);
    pid.setIZone(ks.kF);
    pid.setFF(ks.iZone);

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

    @Override
    public void resetEncoder() {
        encoder.setPosition(0);

    }


}