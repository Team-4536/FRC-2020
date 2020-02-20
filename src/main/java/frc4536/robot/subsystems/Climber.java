package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climber extends SubsystemBase {
    private SpeedController m_armMotor, m_winchMotor;
    private DigitalInput m_topLimitSwitch;
    private DigitalInput m_bottomLimitSwitch;
    private Counter m_bottomLimitSwitchCounter;
    private Counter m_topLimitSwitchCounter;

    public Climber(SpeedController armMotor, SpeedController winchMotor, DigitalInput bottomLimitSwitch, DigitalInput topLimitSwitch) {
        m_winchMotor = winchMotor;
        m_armMotor = armMotor;
        m_bottomLimitSwitch = bottomLimitSwitch;
        m_topLimitSwitch = topLimitSwitch;
        m_bottomLimitSwitchCounter = new Counter(m_bottomLimitSwitch);
        m_topLimitSwitchCounter = new Counter(m_topLimitSwitch);
        m_armMotor.setInverted(true);
        m_bottomLimitSwitchCounter.reset();
        m_topLimitSwitchCounter.reset();
    }

    public boolean bottomLimitSwitchIsSet() {
        return m_bottomLimitSwitchCounter.get() > 0;
    }

    public boolean topLimitSwitchIsSet() {
        return m_topLimitSwitchCounter.get() > 0;
    }

    public boolean climberIsUp(){
        
    }

    public void setArm(double speed) {
        if (bottomLimitSwitchIsSet()) {
            if (speed > 0) {
                m_armMotor.set(speed);
                m_bottomLimitSwitchCounter.reset();
            } else {
                m_armMotor.set(0);
            }
        } else if (topLimitSwitchIsSet()) {
            if (speed < 0) {
                m_armMotor.set(speed);
                m_bottomLimitSwitchCounter.reset();
            } else {
                m_armMotor.set(0);
            }
        } else {
            m_armMotor.set(-speed); //Negative voltage raises the arm.
        }
    }

    public void setWinch(double speed) {
        m_winchMotor.set(speed);
    }
}
