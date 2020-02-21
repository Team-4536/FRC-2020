package frc4536.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4536.robot.Constants;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.Shooter;

public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(Conveyor conveyor, Shooter shooter, DoubleSupplier topRPS, DoubleSupplier bottomRPS) {
      addCommands(
        shooter.spinUp(topRPS, bottomRPS),
        new WaitUntilCommand(shooter::ready).andThen(new ParallelCommandGroup(
          new RunCommand(conveyor::lowerTop),
          new RunCommand(() -> conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED))
        ))
      );
    }

    public ShootCommand(Shooter shooter, Conveyor conveyor) {
      addCommands(
        shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM),
        new WaitUntilCommand(shooter::ready).andThen(new ParallelCommandGroup(
          new RunCommand(conveyor::lowerTop),
          new RunCommand(() -> conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED))
        ))
      );
    }

    public ShootCommand(Shooter shooter, Conveyor conveyor, boolean unsafe) {
      addCommands(
        shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM),
        new WaitCommand(Constants.SHOOT_TIME).andThen(new ParallelCommandGroup(
          new RunCommand(conveyor::lowerTop),
          new RunCommand(() -> conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED))
        ))
      );
    }
}