package frc4536.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4536.robot.Constants;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Intake;
import frc4536.robot.subsystems.Shooter;

import java.util.ArrayList;

public class CycleCommand extends SequentialCommandGroup {
  public CycleCommand(DriveTrain driveTrain, Shooter shooter, Conveyor conveyor) {
    Pose2d shootPosition = new Pose2d(6.0, -0.75, new Rotation2d(4.8, 0.8));
    Pose2d loadingStationPosition = new Pose2d(10.0, -0.75, new Rotation2d(4.8, -0.8)); // I have *no* idea where this is, check in weaver
    Trajectory toShoot = TrajectoryGenerator.generateTrajectory(driveTrain.getPose(), new ArrayList<Translation2d>(), shootPosition, driveTrain.getConfig().setReversed(true));
    Trajectory toLoad = TrajectoryGenerator.generateTrajectory(shootPosition, new ArrayList<Translation2d>(), loadingStationPosition, driveTrain.getConfig().setReversed(false));
    addRequirements(driveTrain, shooter, conveyor);
    addCommands(
      driveTrain.scurveTo(toShoot),
      new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)),
      new ShootCommand(shooter, conveyor, 0.0).withTimeout(3),
      driveTrain.scurveTo(toLoad)
      );
  }
}
