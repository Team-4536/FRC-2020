package frc4536.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class NEOSmartMotor implements ISmartMotor {
private CANSparkMax motor;

public NEOSmartMotor(int deviceID){
    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
}
    @Override
    public void set(double speed) {
        
    }

    @Override
    public double get() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setInverted(boolean isInverted) {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void disable() {
        // TODO Auto-generated method stub

    }

    @Override
    public void stopMotor() {
        // TODO Auto-generated method stub

    }

    @Override
    public void pidWrite(double output) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setVolt(double i) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setSpeed(double i) {
        // TODO Auto-generated method stub

    }

    @Override
    public double getSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getDistance() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getSetpoint() {
        // TODO Auto-generated method stub
        return 0;
    }


}