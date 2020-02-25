package frc4536.lib;

public class Utilities {
    public static double angleConverter(double m_angle) {
        m_angle = m_angle % 360;
        if (m_angle > 180) {
            m_angle = m_angle - 360;
        }
        if (m_angle < -180) {
            m_angle = m_angle + 360;
        }
        return (m_angle);
    }

    public static double deadzone(double value, double limit) {
        if(Math.abs(value)>limit) {
            return value;
        }
        return 0;
    }
}