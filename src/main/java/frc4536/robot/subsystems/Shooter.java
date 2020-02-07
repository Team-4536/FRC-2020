/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc4536.lib.IEncoderMotor;

public class Shooter extends PIDSubsystem {
    private IEncoderMotor m_shooterTop;
    private IEncoderMotor m_shooterBottom;
    //TODO: consider adding feed forward
  /**
   * Creates a new Shooter.
   */
  public Shooter(IEncoderMotor top, IEncoderMotor bottom) {
    //TODO: constants
    super(new PIDController(0.1, 0.1, 0));
    m_shooterTop = top;
    m_shooterBottom = bottom;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  
  public void setRPS(double speed){
    setSetpoint(speed);
  }

  public void setPower(double power) {
    m_shooterTop.setVoltage(power);
    m_shooterBottom.setVoltage(power);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO May need feed forward
    m_shooterTop.setVoltage(output);
    m_shooterBottom.setVoltage(output);

  }

  @Override
  protected double getMeasurement() {
    return m_shooterTop.getSpeed();
  }

}
