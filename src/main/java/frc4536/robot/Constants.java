package frc4536.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Constants {
    private static ShuffleboardTab tab = Shuffleboard.getTab("Constants");

    private static final double SHOOTER_P = 1.0;
    private static final double SHOOTER_KS = 1.0; //Volts to
    private static final double SHOOTER_KV = 1.0;

    private static NetworkTableEntry shooter_p = tab.add("Shooter P", SHOOTER_P).getEntry();
    private static NetworkTableEntry shooter_ks = tab.add("Shooter kS", SHOOTER_KS).getEntry();
    private static NetworkTableEntry shooter_kv = tab.add("Shooter kV", SHOOTER_KV).getEntry();

    public static double getShooterP(){
        return shooter_p.getDouble(SHOOTER_P);
    }

    public static double getShooterkS(){
        return shooter_ks.getDouble(SHOOTER_KS);
    }

    public static double getShooterkV(){
        return shooter_kv.getDouble(SHOOTER_KV);
    }
}
