package frc4536.robot.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import frc4536.lib.ISmartMotor;
import frc4536.lib.NEOSmartMotor;
import frc4536.lib.VirtualMotor;
import frc4536.lib.VirtualSmartMotor;

public class Trenchy implements RobotFrame {

    VirtualSmartMotor m_topFlywheel = new VirtualSmartMotor(3);
    VirtualSmartMotor m_bottomFlywheel = new VirtualSmartMotor(4);
    VirtualMotor fakeIntakeMotor = new VirtualMotor(5);
    VirtualMotor fakeBeltMotor = new VirtualMotor(6);
    AHRS navx = new AHRS();
    NEOSmartMotor leftMotors = new NEOSmartMotor(new double[]{5e-5, 1e-6,0,0},
        0,1);
    NEOSmartMotor rightMotors = new NEOSmartMotor(new double[]{5e-5, 1e-6,0,0},
        2,3);

    @Override
    public ISmartMotor getDrivetrainRightMotor() {
        // TODO Auto-generated method stub
        return rightMotors;
    }

    @Override
    public ISmartMotor getDrivetrainLeftMotor() {
        // TODO Auto-generated method stub
        return leftMotors;
    }

    @Override
    public SpeedController getIntakeMotor() {
        // TODO Auto-generated method stub
        return fakeIntakeMotor;
    }

    @Override
    public SpeedController getBeltMotor() {
        // TODO Auto-generated method stub
        return fakeBeltMotor;
    }
  
    @Override
    public AHRS getDrivetrainNavX() {
        // TODO Auto-generated method stub
        return navx;
    }

    @Override
    public ISmartMotor getTopShooterFlywheelMotor() {
        return m_topFlywheel;
    }

    @Override
    public ISmartMotor getBottomShooterFlywheelMotor() {
        return m_bottomFlywheel;
    }

}