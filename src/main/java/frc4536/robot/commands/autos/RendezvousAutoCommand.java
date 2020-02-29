package frc4536.robot.commands.autos;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4536.robot.Constants;
import frc4536.robot.commands.IntakeCommands;
import frc4536.robot.commands.ShootCommand;
import frc4536.robot.commands.VisionToTargetCommand;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Intake;
import frc4536.robot.subsystems.Shooter;

public class RendezvousAutoCommand extends SequentialCommandGroup {
  public RendezvousAutoCommand(Shooter shooter, Conveyor conveyor, DriveTrain driveTrain, Intake intake, Trajectory toRendezShoot, Trajectory shootToRendez, Trajectory RendezToShoot) {
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
    return "Rendezvous Auton";
  }

}
