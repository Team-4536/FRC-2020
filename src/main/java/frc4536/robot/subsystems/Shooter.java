/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.*;
import frc4536.lib.IEncoderMotor;
import frc4536.robot.Constants;
import frc4536.robot.Dashboard;
import frc4536.robot.Robot;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private IEncoderMotor m_shooterTop;
    private IEncoderMotor m_shooterBottom;
    private PIDController m_topPIDController = new PIDController(Constants.SHOOTER_P_TOP, 0, 0);
    private PIDController m_bottomPIDController = new PIDController(Constants.SHOOTER_P_BOTTOM, 0, 0);
    private SimpleMotorFeedforward k_top_feedForwards = new SimpleMotorFeedforward(Constants.SHOOTER_TOP_KS, Constants.SHOOTER_TOP_KV);
    private SimpleMotorFeedforward k_bottom_feedForwards = new SimpleMotorFeedforward(Constants.SHOOTER_BOTTOM_KS, Constants.SHOOTER_BOTTOM_KV);

    /**
     * Creates a new Shooter.
     */
    public Shooter(IEncoderMotor top, IEncoderMotor bottom) {
        m_shooterTop = top;
        m_shooterBottom = bottom;
        m_topPIDController.setTolerance(Constants.SHOOTER_TOLERANCE_TOP);
        m_bottomPIDController.setTolerance(Constants.SHOOTER_TOLERANCE_BOTTOM);
    }

    public void setTopPower(double power) {
        m_shooterTop.setVoltage(power);
    }

    public void setBottomPower(double power) {
        m_shooterBottom.setVoltage(power);
    }

    public void stop() {
        m_topPIDController.reset();
        m_bottomPIDController.reset();
        m_topPIDController.setSetpoint(0);
        m_bottomPIDController.setSetpoint(0);
        m_shooterTop.set(0);
        m_shooterBottom.set(0);
    }

    public double getTopRate() {
        return m_shooterTop.getSpeed();
    }

    public double getBottomRate() {
        return m_shooterBottom.getSpeed();
    }

    public boolean topReady() {
        return Math.abs(m_topPIDController.getSetpoint() - getTopRate()) < Constants.SHOOTER_TOLERANCE_TOP;
    }

    public boolean bottomReady() {
        return Math.abs(m_bottomPIDController.getSetpoint() - getBottomRate()) < Constants.SHOOTER_TOLERANCE_BOTTOM;
    }

    public boolean ready() {
        return topReady() && bottomReady();
    }

    /**
     * This returns a command that sets the setpoint of the two shooter wheels. THIS SHOULD ONLY BE USED IN AUTO!!!
     *
     * @param topRPS    desired rotation speed of the top shooter wheel, in rotations per second.
     * @param bottomRPS desired rotation speed of the bottom shooter wheel, in rotations per second.
     * @return A command that runs the shooter wheels
     */
    public Command spinUp(DoubleSupplier topRPS, DoubleSupplier bottomRPS) {
        return new RunCommand(() -> setSetpoints(topRPS, bottomRPS), this);
    }

    /**
     * This returns a command that sets the setpoint of the two shooter wheels to the values specified on the Shuffleboard. THIS SHOULD BE USED FOR BUTTON BINDINGS IN TELEOP!!!
     *
     * @return A command that runs the shooter wheels
     */
    public Command spinUp() {
        return this.spinUp(Dashboard::getShooterTopSetpoint, Dashboard::getShooterBottomSetpoint);
    }

    /**
     * This method runs the PID controller code and sets the voltages of the wheels in order to reach a desired speed. This should be called every 20ms, which is why you should only use the commands to run them.
     */
    public void setSetpoints(DoubleSupplier topRPS, DoubleSupplier bottomRPS) {
        m_shooterTop.setVoltage(
                m_topPIDController.calculate(getTopRate(), topRPS.getAsDouble())
                        + k_top_feedForwards.calculate(topRPS.getAsDouble())
        );
        m_shooterBottom.setVoltage(
                m_bottomPIDController.calculate(getBottomRate(), bottomRPS.getAsDouble())
                        + k_bottom_feedForwards.calculate(bottomRPS.getAsDouble())
        );
    }

    /**
     * This method should only be used by the trigger binding in RobotContainer.
     */
    public void setSetpoints() {
        setSetpoints(Dashboard::getShooterTopSetpoint, Dashboard::getShooterBottomSetpoint);
    }


}
