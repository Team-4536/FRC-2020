package frc4536.robot.commands.autos;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4536.robot.Constants;
import frc4536.robot.Poses;
import frc4536.robot.commands.ShootCommand;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

public class InnerAutoCommand extends SequentialCommandGroup {
    public InnerAutoCommand(Shooter shooter, Conveyor conveyor, DriveTrain driveTrain, Pose2d initialPose) {
        Trajectory toInnerShoot = TrajectoryGenerator.generateTrajectory(initialPose, List.of(Poses.INNER_AUTO_WAYPOINT.getTranslation()), Poses.INNER_AUTO_END, driveTrain.getConfig().setReversed(false));
        addCommands(
            driveTrain.scurveTo(toInnerShoot).raceWith(shooter.spinUp(() -> Constants.AUTO_SHOOTER_RPS_TOP, () -> Constants.AUTO_SHOOTER_RPS_BOTTOM)),
            new ShootCommand(shooter, conveyor, 0.0).withTimeout(5)
    );

    }
}