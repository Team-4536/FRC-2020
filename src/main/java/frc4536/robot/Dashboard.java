package frc4536.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Shooter;

public class Dashboard {
    private final NetworkTableEntry topSetpoint, bottomSetpoint;

    Dashboard(RobotContainer robotContainer) {
        DriveTrain driveTrain = robotContainer.m_driveTrain;
        Shooter shooter = robotContainer.m_shooter;
        ShuffleboardTab drivetrain_data = Shuffleboard.getTab("Drivetrain Data");
        drivetrain_data.addNumber("Left Velocity", () -> driveTrain.getSpeeds().leftMetersPerSecond);
        drivetrain_data.addNumber("Right Velocity", () -> driveTrain.getSpeeds().rightMetersPerSecond);
        drivetrain_data.addString("Pose", () -> driveTrain.getPose().toString());
        drivetrain_data.addNumber("Heading", () -> driveTrain.getHeading().getDegrees());
        drivetrain_data.add("Reset Encoders", new InstantCommand(driveTrain::resetEncoders));
        drivetrain_data.add("Reset Pose", new InstantCommand(driveTrain::resetPose));
        drivetrain_data.add("Reset Gyro", new InstantCommand(driveTrain::resetGyro));

        ShuffleboardTab shooter_data = Shuffleboard.getTab("Shooter Data");
        shooter_data.addNumber("Top RPS", shooter::getTopRate);
        shooter_data.addNumber("Bottom RPS", shooter::getBottomRate);
        shooter_data.addBoolean("Top Target", shooter::topReady);
        shooter_data.addBoolean("Bottom Target", shooter::bottomReady);
        topSetpoint = shooter_data.add("Top Setpoint", Constants.SHOOTER_RPS_TOP).getEntry();
        bottomSetpoint = shooter_data.add("Bottom Setpoint", Constants.SHOOTER_RPS_BOTTOM).getEntry();
    }

    public static double getShooterTopSetpoint() {
        return Robot.m_dash == null ? Constants.SHOOTER_RPS_TOP : Robot.m_dash.topSetpoint.getDouble(Constants.SHOOTER_RPS_TOP);
    }

    public static double getShooterBottomSetpoint() {
        return Robot.m_dash == null ? Constants.SHOOTER_RPS_BOTTOM : Robot.m_dash.bottomSetpoint.getDouble(Constants.SHOOTER_RPS_BOTTOM);
    }
}
