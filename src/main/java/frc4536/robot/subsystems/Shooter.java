/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.IEncoderMotor;

public class Shooter extends SubsystemBase {
    private IEncoderMotor m_shooterTop;
    private IEncoderMotor m_shooterBottom;
  /**
   * Creates a new Shooter.
   */
  public Shooter(IEncoderMotor top, IEncoderMotor bottom) {
    m_shooterTop = top;
    m_shooterBottom = bottom;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  
  public void setRPS(double speed){
    //TODO: IMPLEMENT THE PID
  }

  public void setPower(double power) {
    m_shooterTop.setVoltage(power);
    m_shooterBottom.setVoltage(power);
  }

}
