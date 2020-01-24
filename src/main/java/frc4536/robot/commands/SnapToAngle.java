/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.commands;

import frc4536.lib.Utilities;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * An example command that uses an example subsystem.
 */
public class SnapToAngle extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain m_driveTrain;
  private final double m_goalAngle;
  private final PIDController m_controller = new PIDController(1, 0, 0.3);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SnapToAngle(final DriveTrain driveTrain, double goalAngle) {
    m_driveTrain = driveTrain;
    m_goalAngle = Utilities.angleConverter(goalAngle);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset();
    m_controller.setSetpoint(m_goalAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.arcadeDrive(0.0, m_controller.calculate(m_driveTrain.getHeading()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}