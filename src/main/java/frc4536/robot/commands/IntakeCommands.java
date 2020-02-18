package frc4536.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4536.robot.Constants;
import frc4536.robot.subsystems.Conveyor;
import frc4536.robot.subsystems.Intake;

public class IntakeCommands extends CommandBase {
    private final Intake m_intake;
    private final Conveyor m_conveyor;
  public IntakeCommands(Intake intake, Conveyor conveyor) {
    m_intake = intake;
    m_conveyor = conveyor;
    addRequirements(intake,conveyor);
  }

  @Override
  public void execute() {
      m_intake.intake(Constants.INTAKE_SPINSPEED);
      m_intake.extendIntake();
      m_conveyor.moveConveyor(Constants.CONVEYOR_INTAKE_SPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}