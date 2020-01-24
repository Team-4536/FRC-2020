package frc4536.lib;

public class Utilities {
    public final static double angleConverter(double m_angle) {
        m_angle = m_angle % 360;
        if (m_angle > 180) {
            m_angle = m_angle - 360;
        }
        if (m_angle < -180) {
            m_angle = m_angle + 360;
        }
        return (m_angle);
    }
}