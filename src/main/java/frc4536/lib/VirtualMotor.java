package frc4536.lib;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class VirtualMotor implements SpeedController, Sendable {

    private int m_port;
    private double m_speed = 0;
    private boolean m_inverted = false;
    private String m_name, m_subsystem = "";

    public VirtualMotor(String name, int port){
        System.out.println("Virtual motor " + name + " at port " + port + " created");
        this.m_name = name;
        this.m_port = port;

        Shuffleboard.getTab("Virtual Motors")
                .add(((this.m_name.isEmpty()) ? "Virtual Motor" : this.m_name) + " " + this.m_port, this);
    }

    public VirtualMotor(int port){
        this("", port);
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
     * Gets the name of this {@link Sendable} object.
     *
     * @return Name
     */
    @Override
    public String getName() {
        return m_name;
    }

    /**
     * Sets the name of this {@link Sendable} object.
     *
     * @param name name
     */
    @Override
    public void setName(String name) {
        this.m_name = name;
    }

    /**
     * Sets both the subsystem name and device name of this {@link Sendable} object.
     *
     * @param subsystem subsystem name
     * @param name      device name
     */
    @Override
    public void setName(String subsystem, String name) {
        this.m_subsystem = subsystem;
        this.m_name = name;
    }

    /**
     * Gets the subsystem name of this {@link Sendable} object.
     *
     * @return Subsystem name
     */
    @Override
    public String getSubsystem() {
        return m_subsystem;
    }

    /**
     * Sets the subsystem name of this {@link Sendable} object.
     *
     * @param subsystem subsystem name
     */
    @Override
    public void setSubsystem(String subsystem) {
        this.m_subsystem = subsystem;
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
