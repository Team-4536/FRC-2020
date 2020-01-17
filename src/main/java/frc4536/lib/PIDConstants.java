package frc4536.lib;

public class PIDConstants {

    public PIDConstants(double d, double e, double f) {
        this(d, e, f, 0.0);
    }

    public PIDConstants(double d, double e, double f, double g) {
        this(d,e,f,g,0.0);
    }

    public PIDConstants(double d, double e, double f, double g, double h) {
         kP  =  d;
         kI = e;
         kD = f;
         kF = g;
         iZone = h;
    }
    
    public final double kP,kI, kD, kF, iZone;

}