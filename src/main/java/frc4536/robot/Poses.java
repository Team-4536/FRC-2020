package frc4536.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Poses {
    public static final Pose2d
        TRENCH_START     = new Pose2d(3.1,   -0.75, new Rotation2d(1.0,  0.0)),
        TRENCH_SHOOT     = new Pose2d(5.0,   -0.75, new Rotation2d(4.8,  0.8)),
        TRENCH_END       = new Pose2d(8.0,   -1.0,  new Rotation2d(1.4, -0.7)),
        RENDEZ_SHOOT     = new Pose2d(6.0,   -2.5,  new Rotation2d(1.0,  0.0)),
        RENDEZ_SWEEP     = new Pose2d(6.0,   -2.7,  new Rotation2d(0.3, -0.9)),
        LOADING_STATION  = new Pose2d(15.6,  -2.5,  new Rotation2d(1.0,  0.0)),
        HARD_RESET       = new Pose2d(15.6,  -2.1,  new Rotation2d(1.0,  0.0));
}
