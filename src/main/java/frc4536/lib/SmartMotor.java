package frc4536.lib;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Encoder;

public class SmartMotor implements ISmartMotor {
    private final ArrayList<SpeedController> motors = new ArrayList<>();
    private final Encoder m_encoder;
    private final PIDController controller;

    public SmartMotor(Encoder encoder, PIDController control, SpeedController... marray) {
        for(int i=0; i<marray.length; i++){
            motors.add(marray[i]);
        }
        m_encoder = encoder;
        controller = control;

    }

    @Override
    public void setInverted(boolean inverted) {
        motors.forEach(m -> m.setInverted(inverted));
    }

    @Override
    public void set(double speed) {
        motors.forEach(m -> m.set(speed));
    }

    @Override
    public void disable() {
        motors.forEach(m -> m.disable());
    }

    @Override
    public double get() {
        return motors.get(0).get();
    }

    @Override
    public boolean getInverted() {
        return motors.get(0).getInverted();
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
        setVolt(controller.calculate(m_encoder.getRate(), i));

    }

    @Override
    public double getSpeed(){
       return m_encoder.getRate();
    }

    @Override
    public double getDistance() {
        return m_encoder.getDistance();
    }

    @Override
    public double getSetpoint() {
        return controller.getSetpoint();
    }

    @Override
    public void resetEncoder() {
       m_encoder.reset();

    }


}
