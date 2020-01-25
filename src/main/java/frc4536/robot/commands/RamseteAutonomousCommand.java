/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4536.robot.hardware.IRobotConstants.AutoConstants;
import frc4536.robot.hardware.IRobotConstants.DriveConstants;
import frc4536.robot.subsystems.DriveTrain;

/**
 * An example command that uses an example subsystem.
 */
public class RamseteAutonomousCommand extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Trajectory m_trajectory;
  DriveTrain m_driveTrain;
  TrajectoryConfig m_config;
  DifferentialDriveKinematics kDriveKinematics;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RamseteAutonomousCommand(DriveTrain driveTrain, Trajectory trajectory, TrajectoryConfig config) {
    m_trajectory = trajectory;
    m_driveTrain = driveTrain;
    m_config = config;
    addRequirements(m_driveTrain);
    addCommands(
      new RamseteCommand(
          m_trajectory,
          m_driveTrain::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          m_driveTrain::getWheelSpeeds,
          new PIDController(DriveConstants.kPDriveVel, 0, 0), 
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          (left,right) -> m_driveTrain.setVoltages(left,right),
          m_driveTrain
      ).andThen(() -> m_driveTrain.setVoltages(0, 0))
    );
  }
}