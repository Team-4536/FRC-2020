package frc4536.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc4536.robot.lib.SmartMotor;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Robot extends TimedRobot {

  SpeedControllerGroup leftMotors = new SpeedControllerGroup(new Spark(0), new Spark(1)),
  rightMotors = new SpeedControllerGroup(new Spark(2), new Spark(3));

  SmartMotor left = new SmartMotor(new Encoder(0,1), new PIDController(0.1,0,0),leftMotors, 2048,8.0);
  SmartMotor right = new SmartMotor(new Encoder(2,3), new PIDController(0.1,0,0),rightMotors, 2048,8.0);
  DifferentialDrive drive = new DifferentialDrive(left,right);
  AHRS navx = new AHRS();
  XboxController x = new XboxController(0);

  


  @Override
  public void robotInit(){
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-x.getY(Hand.kLeft), x.getX(Hand.kRight));
    SmartDashboard.putNumber("Heading", navx.getYaw());
  }

  public void autonomousPeriodic() {
    left.set(left.getController().calculate(left.getEncoder().getDistance(), 10));
    right.set(right.getController().calculate(right.getEncoder().getDistance(), 10));
  }

}
