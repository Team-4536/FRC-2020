package frc4536.robot.hardware;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.util.Units;
import frc4536.lib.*;

public class Honeycomb implements RobotFrame {
    public static final double ksVolts = 0.235;
    public static final double kvVoltSecondsPerMeter = 0.277;
    public static final double kaVoltSecondsSquaredPerMeter = 0.3;
    public static final double kPDriveVel = 11.5 / 12;
    public static final double kTrackWidthMeters = Units.inchesToMeters(21.8685);
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 16;
    public static final double kRamseteZeta = 0.7;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);

    public RobotConstants m_constants = new RobotConstants(ksVolts, 
                                                           kvVoltSecondsPerMeter, 
                                                           kaVoltSecondsSquaredPerMeter, 
                                                           kPDriveVel, 
                                                           kTrackWidthMeters, 
                                                           kMaxSpeedMetersPerSecond, 
                                                           kMaxAccelerationMetersPerSecondSquared, 
                                                           kRamseteB, 
                                                           kRamseteZeta,
            kWheelDiameterMeters);

    IEncoderMotor m_topFlywheel = new BrushedMAX(1, false, 8192, 21);
    //new PIDBrushedMax(1, false, 8192, new PIDConstants(10.2/12,0,0), 21);
    IEncoderMotor m_bottomFlywheel = new BrushedMAX(1, true, 8192, 20);
    //new PIDBrushedMax(1, true, 8192, new PIDConstants(10.3/12,0,0), 20);

    SpeedController m_intakeMotor = new WPI_VictorSPX(1);
    SpeedController m_beltMotor = new WPI_VictorSPX(4);
    SpeedController m_climberArmMotor = new WPI_VictorSPX(2);
    SpeedController m_liftMotor = new WPI_VictorSPX(3);

    AHRS m_navx = new AHRS();

    IEncoderMotor m_leftMotors = new Neo(10.75, 47, 48);
    IEncoderMotor m_rightMotors = new Neo(10.75, 49, 50);
    DigitalInput m_bottomLimitSwitch = new DigitalInput(0);
    DigitalInput m_conveyorBeam = new DigitalInput(1);

    DoubleSolenoid m_conveyorBlocker = new DoubleSolenoid(1,0);
    DoubleSolenoid m_intakeExtender = new DoubleSolenoid(2,3);

    @Override
    public RobotConstants getConstants() {
        return m_constants;
    }

    @Override
    public IEncoderMotor getDrivetrainRightMotor() {
        return m_rightMotors;
    }

    @Override
    public IEncoderMotor getDrivetrainLeftMotor() {
        return m_leftMotors;
    }

    @Override
    public SpeedController getClimberArmMotor() {
        return m_climberArmMotor;
    }

    @Override
    public SpeedController getLiftMotor() {
        return m_liftMotor;
    }

    @Override
    public DoubleSolenoid getConveyorBlocker() {
        return m_conveyorBlocker;
    }

    @Override
    public DoubleSolenoid getIntakeExtender() {
        return m_intakeExtender;
    }

    @Override
    public SpeedController getIntakeMotor() {
        return m_intakeMotor;
    }

    @Override
    public SpeedController getBeltMotor() {
        return m_beltMotor;
    }
  
    @Override
    public AHRS getDrivetrainNavX() {
        return m_navx;
    }
  
    @Override
    public IEncoderMotor getTopShooterFlywheelMotor() {
        return m_topFlywheel;
    }

    @Override
    public IEncoderMotor getBottomShooterFlywheelMotor() {
        return m_bottomFlywheel;
    }
    @Override
    public DigitalInput getBottomLimitSwitch(){
        return m_bottomLimitSwitch;}

    @Override
    public DigitalInput getConveyorBeam() {
        return m_conveyorBeam;
    }
}
