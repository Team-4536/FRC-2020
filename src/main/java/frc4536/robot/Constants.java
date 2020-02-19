package frc4536.robot;

public class Constants {
    public static final double SHOOTER_P_TOP = 0.895 / 12,
            SHOOTER_TOP_KS = 0.736,
            SHOOTER_TOP_KV = 0.124,
            SHOOTER_TOP_KA = 0.0192,

    SHOOTER_P_BOTTOM = 0.92812 / 12,
            SHOOTER_BOTTOM_KS = 0.454,
            SHOOTER_BOTTOM_KV = 0.108,
            SHOOTER_BOTTOM_KA = 0.197,

    SHOOTER_RPS_TOP = 70,
            SHOOTER_RPS_BOTTOM = 70,
            VISION_KP = 0.02,
            VISION_KI = 0.013333333333333334,
            VISION_KD = 0,
            INTAKE_SPINSPEED = -1,
            CONVEYOR_INTAKE_SPEED = 1.0,
            CONVEYOR_SHOOT_SPEED = 0.5,
            SHOOT_TIME = 4, //seconds
            TURN_TOLERANCE_DEG = 5,
            TURN_TOLERANCE_DEG_PER_SEC = 5,
            DRIVE_DEADZONE = 0.15;
}
