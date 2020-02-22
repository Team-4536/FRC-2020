package frc4536.robot;

public class Constants {
    public static final double SHOOTER_P_TOP = 0.895 / 12,
            SHOOTER_TOP_KS = 0.736,
            SHOOTER_TOP_KV = 0.124,
            SHOOTER_TOP_KA = 0.0192,

    SHOOTER_P_BOTTOM = 0.92812 / 12,
            SHOOTER_BOTTOM_KS = 0.736,
            SHOOTER_BOTTOM_KV = 0.124,
            SHOOTER_BOTTOM_KA = 0.197,

    SHOOTER_RPS_TOP = 50,
            SHOOTER_RPS_BOTTOM = 60,
            VISION_KP = 0.007,
            VISION_KI = 0.007,
            VISION_KD = 0,
            INTAKE_SPINSPEED = -0.8,
            CONVEYOR_INTAKE_SPEED = 0.65,
            CONVEYOR_SHOOT_SPEED = 0.5,
            SHOOT_TIME = 4, //seconds
            TURN_TOLERANCE_DEG = 5,
            TURN_TOLERANCE_DEG_PER_SEC = 5,
            DRIVE_DEADZONE = 0.15;
}
