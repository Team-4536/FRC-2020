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

public class TrenchAutoCommand extends SequentialCommandGroup {
  public TrenchAutoCommand(Shooter shooter, Conveyor conveyor, DriveTrain driveTrain, Intake intake) {

    Pose2d startPosition = new Pose2d(3.1, -0.75, new Rotation2d(0));
    Pose2d shootPosition = new Pose2d(5.0, -0.75, new Rotation2d(4.8, 0.8)); //hypothetically, use angle
    Pose2d trenchEndPosition = new Pose2d(8, -1.0, new Rotation2d(1.4, -0.7));

    Trajectory startToShoot = TrajectoryGenerator.generateTrajectory(startPosition, new ArrayList<Translation2d>(), shootPosition, driveTrain.getConfig().setReversed(false));
    Trajectory shootToEnd = TrajectoryGenerator.generateTrajectory(shootPosition, new ArrayList<Translation2d>(), trenchEndPosition, driveTrain.getConfig().setReversed(false));
    Trajectory endToShoot = TrajectoryGenerator.generateTrajectory(trenchEndPosition, new ArrayList<Translation2d>(), shootPosition, driveTrain.getConfig().setReversed(true));

    addRequirements(shooter, conveyor, driveTrain, intake);
    addCommands(
            driveTrain.scurveTo(startToShoot).raceWith(new IntakeCommands(intake, conveyor)),
            new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)),
            new ShootCommand(shooter, conveyor, 0.0).withTimeout(3),
            driveTrain.scurveTo(shootToEnd).raceWith(new IntakeCommands(intake, conveyor)),
            driveTrain.scurveTo(endToShoot),
            new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM)),
            new ShootCommand(shooter, conveyor, 0.0).withTimeout(3)
    );
  }

  @Override
  public String getName() {
    return "Trench Auton";
  }

}
