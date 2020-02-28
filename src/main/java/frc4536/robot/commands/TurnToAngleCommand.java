package frc4536.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc4536.robot.Constants;
import frc4536.robot.subsystems.DriveTrain;

public class TurnToAngleCommand extends PIDCommand {
    public TurnToAngleCommand(DriveTrain driveTrain, double angleSetpoint) {
        super(new PIDController(Constants.VISION_KP, Constants.VISION_KI, Constants.VISION_KD),
                () -> driveTrain.getBearing(),
                () -> angleSetpoint,
                o -> driveTrain.arcadeDrive(0, -o),
                driveTrain);

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(Constants.TURN_TOLERANCE_DEG, Constants.TURN_TOLERANCE_DEG_PER_SEC);
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public String getName() {
        return "Vision to Target Command";
    }
}