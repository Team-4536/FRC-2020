package frc4536.robot.subsystems;


import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
public  SpeedController m_motor1, m_winchMotor;

public Climber( SpeedController motor1, SpeedController winchMotor){
    m_winchMotor = winchMotor;
    m_motor1 = motor1;
  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArm(double speed){
    m_motor1.set(speed);
  
  }

  public void setWinch(double speed){
    m_winchMotor.set(speed);
  }
}
