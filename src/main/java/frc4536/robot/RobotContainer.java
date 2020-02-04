/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc4536.robot.commands.ExampleCommand;
import frc4536.robot.commands.TankDriveCommand;
import frc4536.robot.hardware.RobotFrame;
import frc4536.robot.hardware.TestRobot;
import frc4536.robot.hardware.Trenchy;
import frc4536.robot.subsystems.DriveTrain;
import frc4536.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final RobotFrame m_robotHardware = new TestRobot();
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain(m_robotHardware.getDrivetrainLeftMotor(), 
                                                         m_robotHardware.getDrivetrainRightMotor(), 
                                                         m_robotHardware.getDrivetrainNavX());

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final XboxController m_driveController = new XboxController(0);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    m_driveTrain.setDefaultCommand(new TankDriveCommand(() -> m_driveController.getY(GenericHID.Hand.kLeft), 
                                                        () -> m_driveController.getX(GenericHID.Hand.kLeft), 
                                                        m_driveTrain));
  
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
