/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    private SpeedController m_motor;
    private DoubleSolenoid m_piston;
    private DigitalInput m_beamBreak;

    public Conveyor(SpeedController motor, DoubleSolenoid piston, DigitalInput beamBreak) {
        m_motor = motor;
        m_piston = piston;
        m_beamBreak = beamBreak;
    }

    public void raiseTop() {
        m_piston.set(Value.kForward);
    }

    public void lowerTop() {
        m_piston.set(Value.kReverse);
    }

    public void moveConveyor(double speed, boolean isShooting) {
        if(isBlocked() || isShooting) m_motor.set(speed);
        else m_motor.set(0);
    }

    public boolean isBlocked(){
        return !m_beamBreak.get();
    }
}
