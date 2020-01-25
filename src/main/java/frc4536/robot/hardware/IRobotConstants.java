package frc4536.robot.hardware;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public interface IRobotConstants {
    public static class DriveConstants {
        public static double ksVolts;
        public static double kvVoltSecondsPerMeter;
        public static double kaVoltSecondsSquaredPerMeter;
        public static double kPDriveVel;
        public static double kTrackWidthMeters;
        public static DifferentialDriveKinematics kDriveKinematics; 
    }
    public static class AutoConstants {
        public static double kMaxSpeedMetersPerSecond;
        public static double kMaxAccelerationMetersPerSecondSquared;
        public static double kRamseteB;
        public static double kRamseteZeta;
    }
}