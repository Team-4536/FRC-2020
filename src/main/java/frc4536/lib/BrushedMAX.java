package frc4536.lib;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.EncoderType;

public class BrushedMAX implements IEncoderMotor {
    private final CANSparkMax m_master;
    private final CANEncoder m_encoder;
    private final double k_gearRatio;
    private final int k_ticks;

    public BrushedMAX(double gearRatio, int ticks, int... motorIDs) {
        k_gearRatio = gearRatio;
        k_ticks = ticks;
        m_master = new CANSparkMax(motorIDs[0], CANSparkMaxLowLevel.MotorType.kBrushed);
        m_encoder = m_master.getEncoder(EncoderType.kQuadrature, k_ticks);
        if(motorIDs.length > 1) for(int i = 1; i < motorIDs.length; i++){
            new CANSparkMax(motorIDs[i], CANSparkMaxLowLevel.MotorType.kBrushed).follow(m_master);
        }
    }

    @Override
    public void set(double speed) {
        m_master.set(speed);
    }   

    @Override
    public double get() {
        return m_master.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_master.setInverted(isInverted);
        m_encoder.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return m_master.getInverted();
    }

    @Override
    public void disable() {
        m_master.disable();
    }

    @Override
    public void stopMotor() {
        m_master.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
        m_master.pidWrite(output);
    }

    @Override
    public double getSpeed() {
        return m_encoder.getVelocity() / (60 * k_gearRatio);
    }

    @Override
    public double getDistance() {
        return m_encoder.getPosition() / (k_gearRatio);
    }

    @Override
    public void resetEncoder() {
        m_encoder.setPosition(0);
    }


}