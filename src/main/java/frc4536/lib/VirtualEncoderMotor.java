package frc4536.lib;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VirtualEncoderMotor extends SubsystemBase implements IEncoderMotor, Sendable {

    private final SpeedController m_motor;
    private final Timer m_timer = new Timer();
    private double m_distance, m_prevTime;
    private final double m_maxSpeed;

    public VirtualEncoderMotor(SpeedController motor, double maxSpeed) {
        m_motor = motor;
        m_maxSpeed = maxSpeed;
        m_timer.reset();
        m_timer.start();
    }

    public VirtualEncoderMotor(String motor, double maxSpeed) {
        m_motor = new VirtualMotor(motor);
        m_maxSpeed = maxSpeed;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void periodic(){ //Integrate the speed over time to get position
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;
        m_distance += m_motor.get()*m_maxSpeed*dt;
        m_prevTime = curTime;
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

    @Override
    public void pidWrite(double output) {
        m_motor.pidWrite(output);
    }

    @Override
    public double getSpeed(){
        return m_motor.get()*m_maxSpeed;
    }

    @Override
    public double getDistance() {
        return m_distance;
    }

    @Override
    public void resetEncoder() {
        m_distance = 0;
    }

    /**
     * Initializes this {@link Sendable} object.
     *
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Speed Controller");
        builder.setActuator(true);
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("Voltage", this::get, this::set);
        builder.addDoubleProperty("Distance", this::getDistance, a -> m_distance = a);
    }

}