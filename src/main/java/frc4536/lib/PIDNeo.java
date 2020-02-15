package frc4536.lib;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

public class PIDNeo extends Neo implements IPIDMotor{
    private final CANPIDController m_pidController;
    private double m_setpoint;

    public PIDNeo(double gearRatio, PIDConstants pidConstants, int... motorIDs) {
        super(gearRatio, motorIDs);
        m_pidController = super.m_master.getPIDController(); //Gets the master motor controller from the super class, Neo.java

        m_pidController.setP(pidConstants.kP);
        m_pidController.setI(pidConstants.kI);
        m_pidController.setD(pidConstants.kD);
        m_pidController.setIZone(pidConstants.iZone);
        m_pidController.setFF(pidConstants.kF);
    }

    @Override
    public void setSpeed(double speed) {
        m_setpoint = speed;
        m_pidController.setReference(m_setpoint, ControlType.kVelocity);
    }

    @Override
    public double getSetpoint() {
        return m_setpoint;
    }
}
