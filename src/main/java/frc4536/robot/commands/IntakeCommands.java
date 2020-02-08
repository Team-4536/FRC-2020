package frc4536.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.Intake;

public class IntakeCommands extends CommandBase {
    private final Intake m_intake;
    private final Conveyor m_conveyor;
    private double k_spinSpeed = -1;
    private double k_conveyorSpeed = 0.5;

  /**
   * Creates a command for driving with a controller
   */
  public IntakeCommands(Intake intake, Conveyor conveyor) {
    m_intake = intake;
    m_conveyor = conveyor;
    addRequirements(intake,conveyor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_intake.intake(k_spinSpeed);
      m_intake.extendIntake();
      m_conveyor.moveConveyor(k_conveyorSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}