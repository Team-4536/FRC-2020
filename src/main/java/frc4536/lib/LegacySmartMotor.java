package frc4536.lib;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;

public class LegacySmartMotor implements ISmartMotor {
    //private final ArrayList<SpeedController> motors = new ArrayList<>();
    private final SpeedControllerGroup m_motors;
    private final Encoder m_encoder;
    private final double m_feedForward;
    private final PIDController m_pid;

    public LegacySmartMotor(Encoder encoder, PIDConstants pidConstants, SpeedControllerGroup motors, int ticks, double maxSpeed) {
        m_motors = motors;
        m_encoder = encoder;
        m_feedForward = 1.0/maxSpeed;
        m_encoder.setDistancePerPulse(1.0 / ticks);
        m_encoder.setPIDSourceType(PIDSourceType.kRate);
        m_pid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD, 1.0/maxSpeed, m_encoder, m_motors);
    
        //TODO: THIS IS FOR TESTING ONLY
        SmartDashboard.putData(m_pid);
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
        m_pid.setSetpoint(i);
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
        return m_pid.getSetpoint();
    }

    @Override
    public void resetEncoder() {
       m_encoder.reset();
    }
}
