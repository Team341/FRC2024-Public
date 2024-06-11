// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public TalonFX leftShooter = new TalonFX(Constants.Shooter.LEFT_SHOOTER_PORT, Constants.CANIVORE_NAME);
  public TalonFX rightShooter = new TalonFX(Constants.Shooter.RIGHT_SHOOTER_PORT, Constants.CANIVORE_NAME);
  public TalonFXConfiguration shooterConfigurationLeft;
  public TalonFXConfiguration shooterConfigurationRight;

  private static Shooter instance;
  private double errorLeft = 99999999.;
  private double errorRight = 9999999.;

  /** Get an instance of Intake. */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  public Shooter() {
    errorLeft = 100;
    errorRight =100;
    shooterConfigurationLeft = new TalonFXConfiguration();
    shooterConfigurationLeft.Slot0.kP = Constants.Shooter.SHOOTER_GAINS.kP;
    shooterConfigurationLeft.Slot0.kI = Constants.Shooter.SHOOTER_GAINS.kI;
    shooterConfigurationLeft.Slot0.kD = Constants.Shooter.SHOOTER_GAINS.kD;
    shooterConfigurationLeft.Slot0.kV = 1./90.;

    shooterConfigurationLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfigurationLeft.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    shooterConfigurationRight = new TalonFXConfiguration();
    shooterConfigurationRight.Slot0.kP = Constants.Shooter.SHOOTER_GAINS.kP;
    shooterConfigurationRight.Slot0.kI = Constants.Shooter.SHOOTER_GAINS.kI;
    shooterConfigurationRight.Slot0.kD = Constants.Shooter.SHOOTER_GAINS.kD;
    shooterConfigurationRight.Slot0.kV = 1./90.;

    shooterConfigurationRight.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    shooterConfigurationLeft.MotorOutput.PeakForwardDutyCycle = 1.0;
    shooterConfigurationLeft.MotorOutput.PeakReverseDutyCycle = 0.;
    shooterConfigurationRight.MotorOutput.PeakForwardDutyCycle = 1.0;
    shooterConfigurationRight.MotorOutput.PeakReverseDutyCycle = 0.;

    shooterConfigurationRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    if (Constants.realRobot) {
      shooterConfigurationLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      shooterConfigurationRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    } else
      shooterConfigurationRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Constants.Shooter.SHOOTER_GAINS.logToDashboard();

    leftShooter.getConfigurator().apply(shooterConfigurationLeft);
    rightShooter.getConfigurator().apply(shooterConfigurationRight);

    leftShooter.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterCurrentLimitConfig);
    rightShooter.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterCurrentLimitConfig);

  }

  /**
   * Rotations per second
   * 
   * @param leftSpeed
   * @param rightSpeed
   */
  public void setSpeed(double leftSpeed, double rightSpeed) {
    leftSpeed = Math.max(leftSpeed, 0);
    rightSpeed = Math.max(rightSpeed, 0);
    leftShooter.setControl(new VelocityDutyCycle(leftSpeed));
    rightShooter.setControl(new VelocityDutyCycle(rightSpeed));

    errorLeft = leftShooter.getClosedLoopError().getValueAsDouble();
    errorRight = rightShooter.getClosedLoopError().getValueAsDouble();

  }

  public boolean atSpeed() {
    return Math.abs(errorLeft) <= Constants.Shooter.RPS_THRESHHOLD
        && Math.abs(errorRight) <= Constants.Shooter.RPS_THRESHHOLD;
  }

  public boolean atSpeed(double rps_tolerance) {
    return Math.abs(errorLeft) <= rps_tolerance
        && Math.abs(errorRight) <= rps_tolerance;
  }

  public boolean atSpeedLeft() {
    return Math.abs(errorLeft) <= Constants.Shooter.RPS_THRESHHOLD;
  }

  public boolean atSpeedRight() {
        return Math.abs(errorRight) <= Constants.Shooter.RPS_THRESHHOLD;
  }

  public void setPercentOutput(double leftSpeed, double rightSpeed) {
    leftShooter.set(leftSpeed);
    rightShooter.set(rightSpeed);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // leftShooter.getConfigurator().apply(shooterConfiguration);
    // rightShooter.getConfigurator().apply(shooterConfiguration);
    if (Constants.FMSDETACHED) {
      SmartDashboard.putNumber("Left Shooter RPS", leftShooter.getRotorVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Right Shooter RPS", rightShooter.getRotorVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Left Shooter Error", leftShooter.getClosedLoopError().getValueAsDouble());
      SmartDashboard.putNumber("Right Shooter Error", rightShooter.getClosedLoopError().getValueAsDouble());
      SmartDashboard.putNumber("Left Shooter Volts", leftShooter.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Right Shooter Volts", rightShooter.getMotorVoltage().getValueAsDouble());

    
        Logger.recordOutput("Left Shooter RPS", leftShooter.getRotorVelocity().getValueAsDouble());
     Logger.recordOutput("Right Shooter RPS", rightShooter.getRotorVelocity().getValueAsDouble());
    }
   Logger.recordOutput("Left Shooter Error", leftShooter.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput("Right Shooter Error", rightShooter.getClosedLoopError().getValueAsDouble());


  }
}
