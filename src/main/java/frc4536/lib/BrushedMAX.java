package frc4536.lib;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.EncoderType;

public class BrushedMAX implements IEncoderMotor {
    protected final CANSparkMax m_master; //Set these to protected so the subclass can access them.
    private final CANEncoder m_encoder;
    private final double k_gearRatio;

    public BrushedMAX(double gearRatio, boolean encoderInverted, int ticks, int... motorIDs) {
        k_gearRatio = gearRatio;
        m_master = new CANSparkMax(motorIDs[0], CANSparkMaxLowLevel.MotorType.kBrushed);
        m_master.restoreFactoryDefaults();
        m_master.setIdleMode(CANSparkMax.IdleMode.kCoast);
        if(motorIDs.length > 1) for(int i = 1; i < motorIDs.length; i++){
            new CANSparkMax(motorIDs[i], CANSparkMaxLowLevel.MotorType.kBrushed).follow(m_master);
        }
        m_encoder = m_master.getEncoder(EncoderType.kQuadrature, ticks);
        m_encoder.setInverted(encoderInverted);
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