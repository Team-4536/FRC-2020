package frc4536.lib;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Encoder;

public class SmartMotor implements SpeedController {
    private final SpeedController m_motor;
    private final Encoder m_encoder;

    public SmartMotor(SpeedController motor, Encoder encoder) {
        m_motor = motor;
        m_encoder = encoder;
    }

    @Override
    public void setInverted(boolean inverted) {
        m_motor.setInverted(inverted);
    }

    @Override
    public void set(double speed) {
        m_motor.set(speed);
    }

    @Override
    public void disable() {
        m_motor.disable();
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public boolean getInverted() {
        return m_motor.getInverted();
    }

    @Override
    public void stopMotor() {
        m_motor.stopMotor();
    }

    public Encoder getEncoder() {
        return m_encoder;
    }

    public int getCount() {
        return m_encoder.get();
    }

    public double getRate() {
        return m_encoder.getRate();
    }

    @Override
    public void pidWrite(double output) {
        throw new UnsupportedOperationException("This method is deprecated, use the 2020 system");
    }
}
