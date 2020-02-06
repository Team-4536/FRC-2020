package frc4536.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.IEncoderMotor;

public class Climber extends SubsystemBase {
public IEncoderMotor m_motor1, m_motor2, m_winchMotor;

public Climber(IEncoderMotor motor1, IEncoderMotor motor2, IEncoderMotor winchMotor){
    m_winchMotor = winchMotor;
    m_motor1 = motor1;
    m_motor2 = motor2;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendArm(double speed){
    m_motor1.set(speed);
    m_motor2.set(speed);
  }

  public void retractArm(double speed){
      m_motor1.set(speed);
      m_motor2.set(speed);
  }

  public void pullWinch(double speed){
    m_winchMotor.set(speed);
  }
}
