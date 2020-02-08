package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  public SpeedController m_motor1;
  private DoubleSolenoid m_piston;

  public Intake(SpeedController motor1, DoubleSolenoid piston) {
    m_motor1 = motor1;
    m_piston = piston;
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake(double speed){
    m_motor1.set(speed);
  }

  public void extendIntake(){
    m_piston.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake(){
    m_piston.set(DoubleSolenoid.Value.kReverse);
  }
}
