/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class Shooter extends SubsystemBase {
    //private SmartMotor shooterTop;
    //private SmartMotor shooterBottom;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    //shooterTop = new SmartMotor();
    //shooterBottom = new SmartMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  
  private void eject(double speed){
    
  }

}
