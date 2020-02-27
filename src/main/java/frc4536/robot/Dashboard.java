package frc4536.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.Shooter;

import java.util.Map;

public class Dashboard {
    private final NetworkTableEntry topSetpoint, bottomSetpoint;

    Dashboard(RobotContainer robotContainer) {
        DriveTrain driveTrain = robotContainer.m_driveTrain;
        Shooter shooter = robotContainer.m_shooter;
        ShuffleboardTab driver_display = Shuffleboard.getTab("Driver Display");

        ShuffleboardLayout shooter_data = driver_display
                .getLayout("Shooter", BuiltInLayouts.kList)
                .withSize(2,6)
                .withPosition(0,0);
        shooter_data.addBoolean("Top Shooter Reached Target RPM", shooter::topReady);
        shooter_data.addBoolean("Bottom Shooter Reached Target RPM", shooter::bottomReady);
        shooter_data.addNumber("Top RPS", shooter::getTopRate).withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 100));
        shooter_data.addNumber("Bottom RPS", shooter::getBottomRate).withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 100));
        topSetpoint = shooter_data.add("Top Setpoint", Constants.SHOOTER_RPS_TOP)
                .getEntry();
        bottomSetpoint = shooter_data.add("Bottom Setpoint", Constants.SHOOTER_RPS_BOTTOM)
                .getEntry();

        ShuffleboardLayout drivetrain_data = driver_display
                .getLayout("Drivetrain", BuiltInLayouts.kList)
                .withSize(2,6)
                .withPosition(2,0);
        drivetrain_data.addNumber("Left Velocity", () -> driveTrain.getSpeeds().leftMetersPerSecond);
        drivetrain_data.addNumber("Right Velocity", () -> driveTrain.getSpeeds().rightMetersPerSecond);
        drivetrain_data.add("Heading", robotContainer.m_robotHardware.getDrivetrainNavX());
        drivetrain_data.add("Reset Encoders", new InstantCommand(driveTrain::resetEncoders));
        drivetrain_data.add("Reset Pose", new InstantCommand(driveTrain::resetPose));
        drivetrain_data.add("Reset Gyro", new InstantCommand(driveTrain::resetGyro));
        drivetrain_data.addString("Pose", () -> {
            Pose2d pose = driveTrain.getPose();
            return String.format("X: %.2f, Y: %.2f, R:%.2f, D: %.2f", pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().getRadians(), pose.getRotation().getDegrees());
        });

        driver_display.addBoolean("Conveyor Blocked", robotContainer.m_conveyor::isBlocked).withSize(1,1);

        ShuffleboardTab debug_display = Shuffleboard.getTab("Debug Display");
        debug_display.add(driveTrain)
                .withPosition(0,0);
        debug_display.add(shooter)
                .withPosition(0,1);
        debug_display.add(robotContainer.m_conveyor)
                .withPosition(0,2);
        debug_display.add(robotContainer.m_intake)
                .withPosition(0,3);
        debug_display.add(robotContainer.m_climber)
                .withPosition(0,4);
        debug_display.add(CommandScheduler.getInstance())
                .withPosition(2,0)
                .withSize(3,5);
        debug_display.add(new PowerDistributionPanel())
                .withPosition(5,0)
                .withSize(3,5);
    }

    public static double getShooterTopSetpoint() {
        return Robot.m_dash == null ? Constants.SHOOTER_RPS_TOP : Robot.m_dash.topSetpoint.getDouble(Constants.SHOOTER_RPS_TOP);
    }

    public static double getShooterBottomSetpoint() {
        return Robot.m_dash == null ? Constants.SHOOTER_RPS_BOTTOM : Robot.m_dash.bottomSetpoint.getDouble(Constants.SHOOTER_RPS_BOTTOM);
    }
}
