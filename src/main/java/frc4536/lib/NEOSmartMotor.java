package frc4536.lib;

import java.util.ArrayList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class NEOSmartMotor implements ISmartMotor {
private ArrayList<CANSparkMax> m_motors = new ArrayList<>();
private CANEncoder m_encoder;
private CANPIDController m_controller;
private double m_setpoint;

    public NEOSmartMotor(PIDConstants ks, int ticks, int...deviceID){
        for(int id : deviceID){
            m_motors.add(new CANSparkMax(id, MotorType.kBrushless));
        }
        m_encoder = m_motors.get(0).getEncoder();

        m_encoder.setVelocityConversionFactor(1.0 / ticks);
        m_encoder.setPositionConversionFactor(1.0 / ticks);

        m_controller = m_motors.get(0).getPIDController();

        m_controller.setP(ks.kP);
        m_controller.setI(ks.kI);
        m_controller.setD(ks.kD);
        m_controller.setIZone(ks.kF);
        m_controller.setFF(ks.iZone);

    }

    @Override
    public void set(double speed) {
        m_motors.forEach(m -> m.set(speed));
    }   

    @Override
    public double get() {
        return m_motors.get(0).get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_motors.forEach(m -> m.setInverted(isInverted));
    }

    @Override
    public boolean getInverted() {
        return m_motors.get(0).getInverted();
    }

    @Override
    public void disable() {
        m_motors.forEach(m -> m.disable());
    }

    @Override
    public void stopMotor() {
        m_motors.forEach(m -> m.stopMotor());
    }

    @Override
    public void pidWrite(double output) {
        m_motors.forEach(m -> m.pidWrite(output));
    }

    @Override
    public void setVolt(double i) {
        m_motors.forEach(m -> m.setVoltage(i));
    }

    @Override
    public void setSpeed(double i) {
        m_setpoint = i;
        m_controller.setReference(m_setpoint, ControlType.kVelocity);
    }

    @Override
    public double getSpeed() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getDistance() {
        return m_encoder.getPosition();
    }

    @Override
    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void resetEncoder() {
        m_encoder.setPosition(0);
    }


}