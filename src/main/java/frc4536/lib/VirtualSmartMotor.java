package frc4536.lib;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;

public class VirtualSmartMotor implements ISmartMotor {

    int port = 0;
    
    public VirtualSmartMotor() {
        // TODO Auto-generated constructor stub
    }
    public VirtualSmartMotor(int port) {
        // TODO Auto-generated constructor stub
        this.port = port;
    }

    @Override
    public void set(double speed) {
        // TODO Auto-generated method stub

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

    @Override
    public void resetEncoder() {
        // TODO Auto-generated method stub

    }

}