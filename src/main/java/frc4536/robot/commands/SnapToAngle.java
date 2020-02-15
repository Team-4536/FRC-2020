/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.commands;

import frc4536.lib.Utilities;
import frc4536.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;


public class SnapToAngle extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_goalAngle;
  private final PIDController m_controller = new PIDController(0.0135, 0.01296, 0.001);

  public SnapToAngle(final DriveTrain driveTrain, DoubleSupplier goalAngle) {
    m_driveTrain = driveTrain;
    m_goalAngle = goalAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset();
    m_controller.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_controller.setSetpoint(Utilities.angleConverter(-m_goalAngle.getAsDouble()));
    m_driveTrain.arcadeDrive(0.0, m_controller.calculate(-m_driveTrain.getHeading().getDegrees()));
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

