package frc4536.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.SmartMotor;

public class Climber extends SubsystemBase {
public SmartMotor m_motor1, m_motor2;

public Climber(SmartMotor motor1, SmartMotor motor2){    
    m_motor1 = motor1;
    m_motor2 = motor2;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend(double speed){
    m_motor1.set(speed);
    m_motor2.set(speed);
  }

  public void retract(double speed){
      m_motor1.set(speed);
      m_motor2.set(speed);
  }

}
