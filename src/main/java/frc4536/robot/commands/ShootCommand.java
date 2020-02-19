package frc4536.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4536.robot.Constants;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
    private Timer m_timer = new Timer();
    private final Shooter m_shooter;
    private final Conveyor m_conveyor;

  public ShootCommand(Shooter shooter, Conveyor conveyor) {
    m_shooter = shooter;
    m_conveyor = conveyor;
    addRequirements(m_shooter, m_conveyor);
  }

  @Override
  public void initialize(){
      m_timer.reset();
      m_timer.start();
  }

  @Override
  public void execute() {
    m_conveyor.lowerTop();
    m_conveyor.moveConveyor(Constants.CONVEYOR_SHOOT_SPEED);
    m_shooter.spinUp(() -> Constants.SHOOTER_RPS_TOP, () -> Constants.SHOOTER_RPS_BOTTOM);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(Constants.SHOOT_TIME);
  }

  @Override
    public void end(boolean a){
        m_timer.stop();
  }

  @Override
    public String getName(){
      return "Shooting";
  }
}

/*
public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(Conveyor conveyor, Shooter shooter, DoubleSupplier topRPS, DoubleSupplier bottomRPS) {
        addCommands(
                shooter.spinUp(topRPS, bottomRPS),
                //TODO: and!
                new WaitUntilCommand(() -> (shooter.getTopRate() > 60 || shooter.getBottomRate() > 60))
                    .andThen(new RunCommand(conveyor::lowerTop)),
                new WaitUntilCommand(() -> (shooter.getTopRate() > 60 || shooter.getBottomRate() > 60))
                    .andThen(new RunCommand(() -> conveyor.moveConveyor(1.0)))
        );
    }
}
*/