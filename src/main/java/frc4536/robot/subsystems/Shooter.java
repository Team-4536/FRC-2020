/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4536.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4536.lib.IEncoderMotor;
import frc4536.robot.Constants;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private IEncoderMotor m_shooterTop;
    private IEncoderMotor m_shooterBottom;
    private PIDController m_topPIDController = new PIDController(Constants.getShooterP(), 0,0);
    private PIDController m_bottomPIDController = new PIDController(Constants.getShooterP(), 0,0);
    private SimpleMotorFeedforward k_feedForwards = new SimpleMotorFeedforward(Constants.getShooterkS(), Constants.getShooterkV());

    /**
     * Creates a new Shooter.
     */
    public Shooter(IEncoderMotor top, IEncoderMotor bottom) {
        m_shooterTop = top;
        m_shooterBottom = bottom;
        m_topPIDController.setTolerance(1.6666);
        m_bottomPIDController.setTolerance(1.6666);
        ShuffleboardTab shooter_data = Shuffleboard.getTab("Shooter Data");
        shooter_data.addNumber("Top RPS", () -> m_shooterTop.getSpeed());
        shooter_data.addNumber("Bottom RPS", () -> m_shooterBottom.getSpeed());
        shooter_data.addBoolean("Top Target", () -> m_topPIDController.atSetpoint());
        shooter_data.addBoolean("Bottom Target", () -> m_bottomPIDController.atSetpoint());
    }

    public void setTopPower(double power) {
        m_shooterTop.setVoltage(power * 12);
    }

    public void setBottomPower(double power){
        m_shooterBottom.setVoltage(power * 12);
    }

    public PIDController getTopPIDController() {
        return m_topPIDController;
    }

    public PIDController getBottomPIDController() {
        return m_bottomPIDController;
    }

    public double getTopRate() {
        return m_shooterTop.getSpeed();
    }

    public double getBottomRate() {
        return m_shooterBottom.getSpeed();
    }

    public Command spinToRPM(DoubleSupplier topRPS, DoubleSupplier bottomRPS){
        Command spinTop = new PIDCommand(
                getTopPIDController(),
                this::getTopRate,
                topRPS.getAsDouble(),
                o -> setTopPower(o + k_feedForwards.calculate(topRPS.getAsDouble())),
                this);
        Command spinBottom = new PIDCommand(
                getBottomPIDController(),
                this::getBottomRate,
                bottomRPS.getAsDouble(),
                o -> setBottomPower(o + k_feedForwards.calculate(bottomRPS.getAsDouble())));

        return new WaitUntilCommand(() -> (m_topPIDController.atSetpoint() && m_bottomPIDController.atSetpoint()))
                .deadlineWith(spinTop, spinBottom);
    }
}
