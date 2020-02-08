package frc4536.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4536.robot.subsystems.Intake;

public class DefaultIntakeCommand extends CommandBase{

    public Intake m_intake;

    public DefaultIntakeCommand(Intake intake){
        m_intake = intake;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
          m_intake.intake(0);;
          m_intake.retractIntake();;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

}