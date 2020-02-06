package frc4536.robot.commands;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4536.robot.subsystems.Winch;

public class WinchCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Winch m_winch;
  private final BooleanSupplier m_spinWinchForwards, m_spinWinchBackwards;
  /**
   * Creates a command for driving with a controller
   */
  public WinchCommand(BooleanSupplier spinWinchForwards, BooleanSupplier spinWinchBackwards, Winch winch) {
    m_winch = winch;
    m_spinWinchForwards = spinWinchForwards;
    m_spinWinchBackwards = spinWinchBackwards;
    addRequirements(winch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if button 1 is pressed turn the motor to extend the arm if button 2 is pressed and button 1 is not turn the motor the other direction
  if(m_spinWinchForwards.getAsBoolean()){
    m_winch.turnWinch(0.5);
    
  }
  else if(m_spinWinchBackwards.getAsBoolean()){
    m_winch.turnWinch(-0.5);
  }
  else m_winch.turnWinch(0);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}