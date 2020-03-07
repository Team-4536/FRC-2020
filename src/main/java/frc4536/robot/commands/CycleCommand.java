package frc4536.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4536.robot.Constants;
import frc4536.robot.Poses;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Intake;
import frc4536.robot.subsystems.Shooter;

import java.util.ArrayList;

public class CycleCommand extends SequentialCommandGroup {
  public CycleCommand(DriveTrain driveTrain, Shooter shooter, Conveyor conveyor) {
    Trajectory toShoot = TrajectoryGenerator.generateTrajectory(driveTrain.getPose(), new ArrayList<Translation2d>(), Poses.TRENCH_SHOOT, driveTrain.getConfig().setReversed(false)); //done
    Trajectory toLoad = TrajectoryGenerator.generateTrajectory(Poses.TRENCH_SHOOT, new ArrayList<Translation2d>(), Poses.LOADING_STATION, driveTrain.getConfig().setReversed(true));
    addRequirements(driveTrain, shooter, conveyor);
    addCommands(
      driveTrain.scurveTo(toShoot),
      new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp()),
      new ShootCommand(shooter, conveyor, 0.0).withTimeout(3),
      driveTrain.scurveTo(toLoad)
      );
  }
}
