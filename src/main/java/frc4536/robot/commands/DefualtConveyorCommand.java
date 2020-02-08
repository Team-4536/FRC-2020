package frc4536.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4536.robot.subsystems.Conveyor;

public class DefualtConveyorCommand extends CommandBase{

    public Conveyor m_conveyor;

    public DefualtConveyorCommand(Conveyor conveyor){
        m_conveyor = conveyor;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
          m_conveyor.moveConveyor(0);
          m_conveyor.raiseTop();
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