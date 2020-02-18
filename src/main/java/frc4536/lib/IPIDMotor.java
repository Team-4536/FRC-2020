package frc4536.lib;

public interface IPIDMotor extends IEncoderMotor {
    void setSetpoint(double speed);
    double getSetpoint();
}
