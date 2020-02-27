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

public class DynamicTrenchAuto extends SequentialCommandGroup {
  public DynamicTrenchAuto(Shooter shooter, Conveyor conveyor, DriveTrain driveTrain, Intake intake, Trajectory shootToEnd, Trajectory endToShoot) {

    Pose2d shootPosition = Poses.AUTO_TRENCH_SHOOT; //hypothetically, use angle
    Trajectory startToShoot = TrajectoryGenerator.generateTrajectory(driveTrain.getPose(), new ArrayList<Translation2d>(), shootPosition, driveTrain.getConfig().setReversed(false));
    addRequirements(shooter, conveyor, driveTrain, intake);
    addCommands(
            driveTrain.scurveTo(startToShoot).raceWith(new IntakeCommands(intake, conveyor)),
            new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp()),
            new ShootCommand(shooter, conveyor, 0.0).withTimeout(3),
            driveTrain.scurveTo(shootToEnd).raceWith(new IntakeCommands(intake, conveyor)),
            driveTrain.scurveTo(endToShoot),
            new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp()),
            new ShootCommand(shooter, conveyor, 0.0).withTimeout(3)
    );
  }

  @Override
  public String getName() {
    return "Dynamic Trench Auton";
  }
}
