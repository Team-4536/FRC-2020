package frc4536.lib;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;


import edu.wpi.first.wpilibj.Encoder;

public class SmartMotor implements ISmartMotor {
    //private final ArrayList<SpeedController> motors = new ArrayList<>();
    private final SpeedController m_motors;
    private final Encoder m_encoder;
    private final PIDController m_controller;

    public SmartMotor(Encoder encoder, PIDController controller, SpeedController motors, int ticks) {
        m_motors = motors;
        m_encoder = encoder;
        m_controller = controller;
        m_encoder.setDistancePerPulse(1.0 / ticks);
    }
    
    @Override
    public void setInverted(boolean inverted) {
        m_motors.setInverted(inverted);
        m_encoder.setReverseDirection(inverted);
    }

    @Override
    public void set(double speed) {
        m_motors.set(speed);
    }

    @Override
    public void disable() {
        m_motors.disable();
    }

    @Override
    public double get() {
        return m_motors.get();
    }

    @Override
    public boolean getInverted() {
        return m_motors.getInverted();
    }

    @Override
    public void stopMotor() {
        m_motors.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
       m_motors.pidWrite(output);
    }

    @Override
    public void setVolt(double i) {
        m_motors.setVoltage(i);

    }

    @Override
    public void setSpeed(double i) {
        setVolt(m_controller.calculate(m_encoder.getRate(), i));

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
        return m_controller.getSetpoint();
    }

    @Override
    public void resetEncoder() {
       m_encoder.reset();

    }


}
