package frc4536.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc4536.robot.Constants;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(Conveyor conveyor, Shooter shooter, DoubleSupplier topRPS, DoubleSupplier bottomRPS) {
        addCommands( new RunCommand(() -> conveyor.moveConveyor(0.5)),
                shooter.spinUp(topRPS, bottomRPS),
                new WaitUntilCommand(() -> (shooter.getTopRate() > 60 && shooter.getBottomRate() > 60))
                    .andThen(new RunCommand(conveyor::lowerTop))
        );
    }
}
