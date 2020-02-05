package frc4536.lib;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Encoder;

public class PWMEncoderMotor implements IEncoderMotor {
    private final SpeedController m_motors;
    private final Encoder m_encoder;

    public PWMEncoderMotor(SpeedController motors, Encoder encoder, int ticksPerRevolution) {
        m_motors = motors;
        m_encoder = encoder;
        m_encoder.setDistancePerPulse(1.0 / ticksPerRevolution);
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
    public double getSpeed(){
       return m_encoder.getRate();
    }

    @Override
    public double getDistance() {
        return m_encoder.getDistance();
    }

    @Override
    public void resetEncoder() {
       m_encoder.reset();
    }
}
