package frc4536.robot.lib;

public class PIDConstants {

    public final double kP, kI, kD, kF, iZone;

    /**
     * Used to store values for PID construction 
     * @param P, proportional term
     * @param I, integral term
     * @param D, derivative term
     */
    public PIDConstants(double P, double I, double D) {
        this(P, I, D, 0.0, 0.0);
    }

    /**
     * Used to store values for PID construction 
     * @param P, proportional term
     * @param I, integral term
     * @param D, derivative term
     * @param feedforward, input prediction to increase accuracy
     */
    public PIDConstants(double P, double I, double D, double feedforward) {
        this(P, I, D, feedforward, 0.0);
    }

    /**
     * Used to store values for PID construction 
     * @param P, proportional term
     * @param I, integral term
     * @param D, derivative term
     * @param feedforward, input prediction to increase accuracy
     * @param iDeadbandZone, specifies the range that |error| must be within for the integral constant to take effect
     */
    public PIDConstants(double P, double I, double D, double feedforward, double iDeadbandZone) {
         kP = P;
         kI = I;
         kD = D;
         kF = feedforward;
         iZone = iDeadbandZone;
    }
    

}