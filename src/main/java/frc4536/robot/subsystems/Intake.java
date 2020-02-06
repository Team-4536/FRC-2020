package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.IEncoderMotor;

public class Intake extends SubsystemBase {
  public IEncoderMotor m_motor1;
  public Intake(IEncoderMotor motor1) {
    m_motor1 = motor1;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake(double speed){
      m_motor1.set(speed);
  }
}
