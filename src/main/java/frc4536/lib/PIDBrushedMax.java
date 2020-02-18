package frc4536.lib;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class PIDBrushedMax extends BrushedMAX implements IPIDMotor, Sendable {
    private final CANPIDController m_pidController;
    private double m_setpoint;

    public PIDBrushedMax(double gearRatio, boolean encoderInverted, int ticks, PIDConstants pidConstants, int... motorIDs) {
        super(gearRatio, encoderInverted, ticks, motorIDs);
        m_pidController = super.m_master.getPIDController(); //Gets the master motor controller from the super class, Neo.java

        m_pidController.setP(pidConstants.kP);
        m_pidController.setI(pidConstants.kI);
        m_pidController.setD(pidConstants.kD);
        m_pidController.setIZone(pidConstants.iZone);
        m_pidController.setFF(pidConstants.kF);
    }

    @Override
    public void setSetpoint(double speed) {
        m_setpoint = speed;
        m_pidController.setReference(m_setpoint, ControlType.kVelocity);
    }

    @Override
    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.setActuator(true);
        builder.addDoubleProperty("p", m_pidController::getP, m_pidController::setP);
        builder.addDoubleProperty("i", m_pidController::getI, m_pidController::setI);
        builder.addDoubleProperty("d", m_pidController::getD, m_pidController::setD);
        builder.addDoubleProperty("iZone", m_pidController::getIZone, m_pidController::setIZone);
        builder.addDoubleProperty("f", m_pidController::getFF, m_pidController::setFF);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

}
