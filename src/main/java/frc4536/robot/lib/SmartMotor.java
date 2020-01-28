package frc4536.robot.lib;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;

public class SmartMotor extends PIDSubsystem implements ISmartMotor {
    //private final ArrayList<SpeedController> motors = new ArrayList<>();
    private final SpeedControllerGroup m_motors;
    private final Encoder m_encoder;
    private final PIDController m_controller;
    private final double m_feedForward;

    public SmartMotor(Encoder encoder, PIDController controller, SpeedControllerGroup motors, int ticks, double maxSpeed) {
        super(controller);
        m_motors = motors;
        m_encoder = encoder;
        m_controller = controller;
        m_feedForward = 1.0/maxSpeed;
        m_encoder.setDistancePerPulse(1.0 / ticks);
        super.enable();
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
        setSetpoint(i);
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

    @Override
    protected void useOutput(double output, double setpoint) {
        this.setVoltage(output + m_feedForward);
    }

    @Override
    protected double getMeasurement() {
        return getSpeed();
    }

    


}
