// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX mLeftClimbMotor;
  TalonFX mRightClimbMotor;
  public boolean isClimbed = false;
  public boolean beenZeroed = false;
  DigitalInput mRightLimitSwitch;
  DigitalInput mLeftLimitSwitch;

  public Climber() {
    mLeftClimbMotor = new TalonFX(Constants.Climber.LEFT_MOTOR, Constants.CANIVORE_NAME);
    mRightClimbMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR, Constants.CANIVORE_NAME);
    var climbMotorConfig = new TalonFXConfiguration();
    climbMotorConfig.Slot0.kP = 1.0;
    climbMotorConfig.Slot0.kI = 0.;
    climbMotorConfig.Slot0.kD = 0.;

    climbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    climbMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    climbMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    mLeftClimbMotor.getConfigurator().apply(climbMotorConfig);
    mRightClimbMotor.getConfigurator().apply(climbMotorConfig);

    mLeftClimbMotor.setInverted(true);

    mRightLimitSwitch = new DigitalInput(Constants.Climber.RIGHT_LIMIT_SWITCH);
    mLeftLimitSwitch = new DigitalInput(Constants.Climber.LEFT_LIMIT_SWITCH);

  }

  private static Climber instance;

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  public void setPosition(double position) {
    mLeftClimbMotor.setControl(new PositionDutyCycle(position));

    mRightClimbMotor.setControl(new PositionDutyCycle(position));
  }

  public boolean atPosition() {
    return Math.abs(getPositionLeft() - 140.) < 5 && Math.abs(getPositionRight() - 140) < 5;
  }

  public void setSpeedLeft(double speed) {
    if (speed < 0 && getLeftLimit()) {
      mLeftClimbMotor.set(0);
      return;
    }
    mLeftClimbMotor.set(speed);
  }

  public double getPositionLeft() {
    return mLeftClimbMotor.getPosition().getValueAsDouble();
  }

  public double getPositionRight() {
    return mRightClimbMotor.getPosition().getValueAsDouble();

  }

  public boolean getLeftLimit() {
    return mLeftLimitSwitch.get();
  }

  public boolean getRightLimit() {
    return mRightLimitSwitch.get();
  }

  public void setSpeedRight(double speed) {
    if (speed < 0 && getRightLimit()) {
      mRightClimbMotor.set(0);
      return;
    }
    mRightClimbMotor.set(speed);
  }

  public void setEncoderRight(double position) {
    mRightClimbMotor.setPosition(position);

  }

  public void setEncoderLeft(double position) {
    mLeftClimbMotor.setPosition(position);

  }

  @Override
  public void periodic() {
    if (getLeftLimit() && getRightLimit()) {
      beenZeroed = true;
    }
    if (getLeftLimit() && Math.abs(getPositionLeft()) > 0.5) {
      setEncoderLeft(0);
    }
    if (getRightLimit() && Math.abs(getPositionRight()) > 0.5) {
      setEncoderRight(0);
    }
    if (Constants.FMSDETACHED) {
      SmartDashboard.putNumber("Left Climb Encoder", getPositionLeft());
      SmartDashboard.putNumber("Right Climb Encoder", getPositionRight());
      SmartDashboard.putBoolean("Climber is Climbed", isClimbed);

      SmartDashboard.putBoolean("Right Climber Limit Switch", getRightLimit());
      SmartDashboard.putBoolean("Left Climber Limit Switch", getLeftLimit());
    }

  }
}
