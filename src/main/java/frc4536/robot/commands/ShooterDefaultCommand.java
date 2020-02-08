/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.commands;

import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ShooterDefaultCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Shooter m_shooter;
    private final DoubleSupplier m_speed;
    private double m_maxRPS;

    /**
     * Creates a command for driving with a controller
     * 
     * @param speed,     supplier for speed (usually from trigger)
     * @param maxRPS,    what do you think?
     * @param shooter    The subsystem used by this command.
     */
    public ShooterDefaultCommand(DoubleSupplier speed, double maxRPS, Shooter shooter) {
        m_shooter = shooter;
        m_speed = speed;
        m_maxRPS = maxRPS;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.setRPS(m_maxRPS * m_speed.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setRPS(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
