package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import frc4536.lib.ISmartMotor;
import frc4536.lib.NEOSmartMotor;
import frc4536.lib.VirtualMotor;

public class Trenchy implements RobotFrame {
    VirtualMotor fakeflywheelmotor = new VirtualMotor(4);
    VirtualMotor fakeintakemotor = new VirtualMotor(5);
    VirtualMotor fakebeltmotor = new VirtualMotor(6);
    AHRS navx = new AHRS();
    NEOSmartMotor leftmotors = new NEOSmartMotor(new double[]{5e-5, 1e-6,0,0},
        0,1);
    NEOSmartMotor rightmotors = new NEOSmartMotor(new double[]{5e-5, 1e-6,0,0},
        2,3);


    @Override
    public ISmartMotor getDrivetrainRightMotor() {
        // TODO Auto-generated method stub
        return rightmotors;
    }

    @Override
    public ISmartMotor getDrivetrainLeftMotor() {
        // TODO Auto-generated method stub
        return leftmotors;
    }

    @Override
    public SpeedController getShooterFlywheelMotor() {
        // TODO Auto-generated method stub
        return fakeflywheelmotor;
    }

    @Override
    public SpeedController getIntakeMotor() {
        // TODO Auto-generated method stub
        return fakeintakemotor;
    }

    @Override
    public SpeedController getBeltMotor() {
        // TODO Auto-generated method stub
        return fakebeltmotor;
    }

    @Override
    public AHRS getDrivetrainNavX() {
        // TODO Auto-generated method stub
        return navx;
    }

}