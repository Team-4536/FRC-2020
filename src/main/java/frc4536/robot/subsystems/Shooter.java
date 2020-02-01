/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.ISmartMotor;

public class Shooter extends SubsystemBase {
    private ISmartMotor m_shooterTop;
    private ISmartMotor m_shooterBottom;
  /**
   * Creates a new Shooter.
   */
  public Shooter(ISmartMotor top, ISmartMotor bottom) {
    m_shooterTop = top;
    m_shooterBottom = bottom;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  
  public void setRPS(double speed){
    m_shooterTop.setSpeed(speed);
    m_shooterBottom.setSpeed(-speed);
  }

}
