package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Winch extends SubsystemBase {
    private final SpeedController m_winchMotor;
    public Winch(SpeedController winchMotor) {
        m_winchMotor = winchMotor;
    }

    public void turnWinch(double speed) {
        m_winchMotor.set(speed);
    }
}