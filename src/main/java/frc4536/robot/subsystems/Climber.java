package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climber extends SubsystemBase {
    private SpeedController m_armMotor, m_winchMotor;
    private DigitalInput m_bottomLimitSwitch;
    public Climber(SpeedController armMotor, SpeedController winchMotor, DigitalInput bottomLimitSwitch) {
        m_winchMotor = winchMotor;
        m_armMotor = armMotor;
        m_bottomLimitSwitch = bottomLimitSwitch;
        m_armMotor.setInverted(true); //TODO: Test climber direction
    }
    public boolean bottomLimitSwitchIsSet() {
      return !m_bottomLimitSwitch.get();
  }

    public void setArm(double speed) {
        
        if (bottomLimitSwitchIsSet()){
         if(speed>0){
           m_armMotor.set(speed);
        }
           else {m_armMotor.set(0);
          }

        }
        else {
          m_armMotor.set(speed); //Negative voltage raises the arm.
        }
    }

    public void setWinch(double speed) {
        m_winchMotor.set(speed);
    }
}
