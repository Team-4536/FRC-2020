package frc4536.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Robot extends TimedRobot {

  DifferentialDrive drive = new DifferentialDrive(
    new SpeedControllerGroup(new Spark(0), new Spark(1)),
    new SpeedControllerGroup(new Spark(2), new Spark(3))
  );

  Encoder left = new Encoder(0,1), right = new Encoder(2,3);

  Joystick j = new Joystick(0);

  @Override
  public void robotInit(){
    SmartDashboard.putData(left);
    SmartDashboard.putData(right);
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-j.getY(), j.getX());
  }
}
