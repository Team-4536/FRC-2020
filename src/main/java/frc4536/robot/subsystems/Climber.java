package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private SpeedController m_armMotor, m_winchMotor;

    public Climber(SpeedController motor1, SpeedController winchMotor) {
        m_winchMotor = winchMotor;
        m_armMotor = motor1;
    }

    public void setArm(double speed) {
        m_armMotor.set(-speed); //Negative voltage raises the arm.
    }

    public void setWinch(double speed) {
        m_winchMotor.set(speed);
    }
}
