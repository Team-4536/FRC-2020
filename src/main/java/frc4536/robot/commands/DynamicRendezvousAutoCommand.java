package frc4536.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
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

public class DynamicRendezvousAutoCommand extends SequentialCommandGroup {
    public DynamicRendezvousAutoCommand(Shooter shooter, Conveyor conveyor, DriveTrain driveTrain, Intake intake, Trajectory shootToRendez, Trajectory RendezToShoot) {
        Pose2d shootPosition = Poses.RENDEZ_SHOOT;
        Trajectory toRendezShoot = TrajectoryGenerator.generateTrajectory(driveTrain.getPose(), new ArrayList<Translation2d>(), shootPosition, driveTrain.getConfig().setReversed(false));
        addRequirements(shooter, conveyor, driveTrain, intake);
        addCommands(
                driveTrain.scurveTo(toRendezShoot).raceWith(new IntakeCommands(intake, conveyor)),
                new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp()),
                new ShootCommand(shooter, conveyor, 0.0).withTimeout(3),
                driveTrain.scurveTo(shootToRendez).raceWith(new IntakeCommands(intake, conveyor)),
                driveTrain.scurveTo(RendezToShoot),
                new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp()),
                new ShootCommand(shooter, conveyor, 0.0).withTimeout(3)
        );
    }

    @Override
    public String getName() {
        return "Dynamic Rendezvous Auton";
    }

}
