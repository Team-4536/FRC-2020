/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    /**
    * Creates a new ExampleSubsystem.
    */
    public SpeedController m_motor;
    public DoubleSolenoid m_piston;

    public Conveyor(SpeedController motor, DoubleSolenoid piston) {
        super();
        m_motor = motor;
        m_piston = piston;
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    public void raiseTop(){
        m_piston.set(Value.kForward);
    }

    public void lowerTop(){
        m_piston.set(Value.kReverse);
    }

    public void moveConveyor(double speed){
        m_motor.set(speed);
    }
    
}
