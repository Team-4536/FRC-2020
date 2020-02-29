package frc4536.robot.commands.autos;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4536.robot.commands.ShootCommand;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Shooter;

public class CenterAutoCommand extends SequentialCommandGroup {
    public CenterAutoCommand(Shooter shooter, Conveyor conveyor, DriveTrain driveTrain, Trajectory trajectory) {
        addRequirements(shooter, conveyor, driveTrain);
        addCommands(
                driveTrain.scurveTo(trajectory).raceWith(shooter.spinUp()),
                new ShootCommand(shooter, conveyor, 0.0).withTimeout(5)
        );
    }
}
