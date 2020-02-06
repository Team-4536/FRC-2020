/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4536.lib.IEncoderMotor;

public class Conveyor extends SubsystemBase {
    /**
    * Creates a new ExampleSubsystem.
    */
    public IEncoderMotor m_topMotor, m_bottomMotor;
    public double k_topSpeed, k_bottomSpeed;
    /*public Piston m_piston*/

    public Conveyor(IEncoderMotor topMotor, IEncoderMotor bottomMotor, double topSpeed, double bottomSpeed /*, Piston piston*/) {
        super();
        m_topMotor = topMotor;
        m_bottomMotor = bottomMotor;
        k_topSpeed = topSpeed;
        k_bottomSpeed = bottomSpeed;
        /*m_piston = piston;*/
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    public void lowerTop(){
        /*
        if(m_piston.isExtended()){
            m_piston.toggleTop();
        }
        */
    }

    public void raiseTop(){
        /*
        if(!m_piston.isExtended()){
            m_piston.toggleTop();
        }
        */
    }

    public void moveConveyor(double topSpeed, double bottomSpeed){
        m_topMotor.set(topSpeed);
        m_bottomMotor.set(bottomSpeed);
    }
    
}
