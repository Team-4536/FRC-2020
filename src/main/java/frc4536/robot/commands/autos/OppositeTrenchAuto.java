package frc4536.robot.commands.autos;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4536.robot.commands.IntakeCommands;
import frc4536.robot.commands.ShootCommand;
import frc4536.robot.commands.VisionToTargetCommand;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Intake;
import frc4536.robot.subsystems.Shooter;

public class OppositeTrenchAuto extends SequentialCommandGroup {
        public OppositeTrenchAuto(Shooter shooter, Conveyor conveyor, DriveTrain driveTrain, Intake intake, Trajectory oppositeTrenchRun, Trajectory oppositeEndToShoot) {
            addRequirements(shooter, conveyor, driveTrain, intake);
            addCommands(
                    new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp()),
                    new ShootCommand(shooter, conveyor, 0.0).withTimeout(3),
                    driveTrain.scurveTo(oppositeTrenchRun).raceWith(new IntakeCommands(intake, conveyor)),
                    driveTrain.scurveTo(oppositeEndToShoot),
                    new VisionToTargetCommand(driveTrain).raceWith(shooter.spinUp()),
                    new ShootCommand(shooter, conveyor, 0.0).withTimeout(3)
            );
        }

        @Override
        public String getName() {
            return "Opposite Trench Auto";
        }

    }

