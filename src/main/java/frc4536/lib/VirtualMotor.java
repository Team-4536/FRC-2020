package frc4536.lib;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class VirtualMotor implements SpeedController, Sendable {
    private double m_speed = 0;
    private boolean m_inverted = false;
    private String m_name;

    public VirtualMotor(String name){
        System.out.println("Virtual motor " + name + " created");
        this.m_name = name;

        //TODO: comtemplate moving shuffleboard interaction
        Shuffleboard.getTab("Virtual Motors").add("Virtual Motor " + this.m_name, this);
    }

    @Override
    public void set(double speed) {
        m_speed = speed;
    }

    @Override
    public double get() {
        return m_speed;
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_inverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return m_inverted;
    }

    @Override
    public void disable() {
        m_speed = 0;
    }

    @Override
    public void stopMotor() {
        m_speed = 0;
    }

    @Override
    public void pidWrite(double output) {
        m_speed = output;
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
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}
