package frc4536.robot.commands;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4536.robot.hardware.RobotConstants;
import frc4536.robot.subsystems.DriveTrain;
import java.util.List;

public class ScurveToLocation extends CommandBase { //TODO: This command is not finished.
    RobotConstants m_constants;
    public ScurveToLocation(RobotConstants constants, DriveTrain driveTrain){
        m_constants = constants;
        addRequirements(driveTrain);

        // Create a voltage constraint to ensure we don't accelerate too fast
        TrajectoryConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(m_constants.ksVolts,
                                m_constants.kvVoltSecondsPerMeter,
                                m_constants.kaVoltSecondsSquaredPerMeter),
                        m_constants.kDriveKinematics,
                        10);
        TrajectoryConfig m_config =
                new TrajectoryConfig(m_constants.kMaxSpeedMetersPerSecond,
                        m_constants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(m_constants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        final Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                m_config
        );
    }

    public void execute() {
    }

    //TODO: Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
