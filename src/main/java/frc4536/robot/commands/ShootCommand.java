package frc4536.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc4536.robot.Constants;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.Shooter;

public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(Shooter shooter, Conveyor conveyor, DoubleSupplier topRPS, DoubleSupplier bottomRPS, double shootTime) {
      Command condition;
      if(shootTime < 0) {
        condition = new WaitUntilCommand(shooter::ready);
      }
      else {
        condition = new WaitCommand(shootTime);
      }
      addRequirements(conveyor);
      addCommands(
        shooter.spinUp(topRPS, bottomRPS),
        condition.andThen(new ParallelCommandGroup(
          new RunCommand(conveyor::lowerTop),
          new RunCommand(() -> conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED, true))
        ))
      );
    }

  /**
   * Waits until shooter reaches supplied RPS to shoot.
   */
  public ShootCommand(Shooter shooter, Conveyor conveyor, DoubleSupplier topRPS, DoubleSupplier bottomRPS) {
    this(shooter, conveyor, topRPS, bottomRPS, -1);
  }

  /**
   *  Waits until @shootTime seconds to shoot.
   */
    public ShootCommand(Shooter shooter, Conveyor conveyor, double shootTime) {
      this(shooter, conveyor, () -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM, shootTime);
    }

    /**
     * Waits until shooter reaches set RPS to shoot.
     */
    public ShootCommand(Shooter shooter, Conveyor conveyor) {
      this(shooter, conveyor, -1);
    }
}