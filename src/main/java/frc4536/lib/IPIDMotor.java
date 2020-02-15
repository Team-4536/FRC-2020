package frc4536.lib;

public interface IPIDMotor extends IEncoderMotor {
    void setSpeed(double speed);
    double getSetpoint();
}
