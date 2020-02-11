package frc4536.robot.hardware;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class RobotConstants {
    public RobotConstants(double ksVolts, 
                          double kvVoltSecondsPerMeter, 
                          double kaVoltSecondsSquaredPerMeter, 
                          double kPDriveVel, 
                          double kTrackWidthMeters, 
                          double kMaxSpeedMetersPerSecond, 
                          double kMaxAccelerationMetersPerSecondSquared,
                          double kRamseteB, 
                          double kRamseteZeta,
                          double kWheelDiameterInches
                          ) {
        this.ksVolts = ksVolts;
        this.kvVoltSecondsPerMeter = kvVoltSecondsPerMeter;
        this.kaVoltSecondsSquaredPerMeter = kaVoltSecondsSquaredPerMeter;
        this.kPDriveVel = kPDriveVel;
        this.kTrackWidthMeters = kTrackWidthMeters;
        kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
        this.kMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond;
        this.kMaxAccelerationMetersPerSecondSquared = kMaxAccelerationMetersPerSecondSquared;
        this.kRamseteB = kRamseteB;
        this.kRamseteZeta = kRamseteZeta;
        this.kWheelDiameterInches = kWheelDiameterInches;
    }
    public final double ksVolts;
    public final double kvVoltSecondsPerMeter;
    public final double kaVoltSecondsSquaredPerMeter;
    public final double kPDriveVel;
    public final double kTrackWidthMeters;
    public final DifferentialDriveKinematics kDriveKinematics;
    public final double kMaxSpeedMetersPerSecond;
    public final double kMaxAccelerationMetersPerSecondSquared;
    public final double kRamseteB;
    public final double kRamseteZeta;
    public final double kWheelDiameterInches;
}