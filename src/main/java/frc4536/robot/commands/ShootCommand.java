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
    public ShootCommand(Shooter shooter, Conveyor conveyor, DoubleSupplier topRPS, DoubleSupplier bottomRPS) {
      addRequirements(conveyor);
      addCommands(
        shooter.spinUp(topRPS, bottomRPS),
        new WaitUntilCommand(shooter::ready).andThen(new ParallelCommandGroup(
          new RunCommand(conveyor::lowerTop),
          new RunCommand(() -> conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED))
        ))
      );
    }

    public ShootCommand(Shooter shooter, Conveyor conveyor) {
      addRequirements(conveyor);
      addCommands(
        shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM),
        new WaitUntilCommand(shooter::ready).andThen(new ParallelCommandGroup(
          new RunCommand(conveyor::lowerTop),
          new RunCommand(() -> conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED))
        ))
      );
    }

    public ShootCommand(Shooter shooter, Conveyor conveyor, double shootTime) {
      addRequirements(conveyor);
      addCommands(
        shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM),
        new WaitCommand(shootTime).andThen(new ParallelCommandGroup(
          new RunCommand(conveyor::lowerTop),
          new RunCommand(() -> conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED))
        ))
      );
    }
}