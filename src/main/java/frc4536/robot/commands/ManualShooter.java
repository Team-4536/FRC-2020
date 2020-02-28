package frc4536.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc4536.robot.Constants;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class ManualShooter extends FunctionalCommand {
    public ManualShooter(DoubleSupplier turnRate, DriveTrain driveTrain, Shooter shooter) {
        super(
                () -> driveTrain.toggleDriverVision(true),
                () -> {
                    driveTrain.arcadeDrive(0, turnRate.getAsDouble() * Constants.MANUAL_SHOOT_TURN);
                    shooter.setSetpoints();
                },
                lmaoxxddddddddd -> driveTrain.toggleDriverVision(false),
                () -> false,
                driveTrain, shooter);
    }
}
