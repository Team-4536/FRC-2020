package frc4536.robot.commands.autos;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4536.robot.Poses;
import frc4536.robot.commands.ShootCommand;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Shooter;

import java.util.ArrayList;

public class CenterAutoCommand extends SequentialCommandGroup {
    public CenterAutoCommand(Shooter shooter, Conveyor conveyor, DriveTrain driveTrain, Pose2d initialPose) {
        Pose2d shootPosition = Poses.CENTER_AUTO_END;
        Trajectory toRendezShoot = TrajectoryGenerator.generateTrajectory(initialPose, new ArrayList<Translation2d>(), shootPosition, driveTrain.getConfig().setReversed(true)); //done
        addRequirements(shooter, conveyor, driveTrain);
        addCommands(
                driveTrain.scurveTo(toRendezShoot).raceWith(shooter.spinUp(() -> 90, () -> 50)),
                new ShootCommand(shooter, conveyor, 0.0).withTimeout(5)
        );
    }
}