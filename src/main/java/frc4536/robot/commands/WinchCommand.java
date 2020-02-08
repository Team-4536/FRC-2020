package frc4536.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4536.robot.subsystems.Climber;

public class WinchCommand extends CommandBase {
  private final BooleanSupplier m_winchTime, m_activateArm;
  private final Climber m_climber;
  private final DoubleSupplier m_armSpeed;
  /**
   * Creates a command for driving with a controller
   */
  public WinchCommand(BooleanSupplier winchTime, DoubleSupplier armSpeed, BooleanSupplier activateArm, Climber climber) {
    m_winchTime = winchTime;
    m_climber = climber;
    m_activateArm = activateArm;
    m_armSpeed = armSpeed;
   // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_activateArm.getAsBoolean()){
      m_climber.setArm(m_armSpeed.getAsDouble());
    } else {
      m_climber.setArm(0);
    }
    if(m_winchTime.getAsBoolean()){
      m_climber.setWinch(m_armSpeed.getAsDouble());
    } else{
      m_climber.setWinch(0);
    }
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}