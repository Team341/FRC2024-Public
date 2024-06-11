// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  CANSparkMax mUptakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_PORT, MotorType.kBrushless); // Falcon on
                                                                                                        // real
  CANSparkMax mConveyorMotor = new CANSparkMax(Constants.Intake.CONVEYOR_MOTOR_PORT, MotorType.kBrushless);
  CANSparkMax mBeltMotor = new CANSparkMax(Constants.Intake.BELT_MOTOR_PORT, MotorType.kBrushless);

  boolean solonoidState;
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Intake.SOLENOID_PORT,
      Constants.Intake.SOLENOID_PORT_OUT);
  DigitalInput conveyorBeamBreak = new DigitalInput(Constants.Intake.CONVEYOR_BEAM_BREAK_PORT);
  DigitalInput intakeBeamBreak = new DigitalInput(Constants.Intake.INTAKE_BEAM_BREAK_PORT);

  public Intake() {
    mUptakeMotor.setInverted(false);
    mBeltMotor.setInverted(false);
    mConveyorMotor.setInverted(true);
    mUptakeMotor.setIdleMode(IdleMode.kBrake);
    mBeltMotor.setIdleMode(IdleMode.kBrake);

    mConveyorMotor.setIdleMode(IdleMode.kBrake);

    mUptakeMotor.setSmartCurrentLimit(60);
    mBeltMotor.setSmartCurrentLimit(30);
    mConveyorMotor.setSmartCurrentLimit(60);

    mConveyorMotor.burnFlash();
    mBeltMotor.burnFlash();

    mUptakeMotor.burnFlash();
    solonoidState = solenoid.get() == DoubleSolenoid.Value.kReverse;

  }

  private static Intake instance;

  public static Intake getInstance() {

    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public void setSpeedIntake(double speed) {
    mUptakeMotor.set(speed);
    mBeltMotor.set(speed);
    mConveyorMotor.set(speed * 0.85);
  }

  public void setSpeedUptake(double speed) {
    mUptakeMotor.set(speed);
    mBeltMotor.set(speed);

  }

  public void deploy() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
    solonoidState =true;

  }

  public void retract() {
    solenoid.set(DoubleSolenoid.Value.kForward);
        solonoidState =false;

  }

  public void toggle() {
    if (solonoidState) {
      solenoid.set(DoubleSolenoid.Value.kForward);
      solonoidState = !solonoidState;
    } else {
      solenoid.set(DoubleSolenoid.Value.kReverse);
      solonoidState = !solonoidState;
    }
  }

  public boolean isDeployed() {
    if (solonoidState)
      return true;
    return false;
  }

  public boolean isInConveyor() {
    return !conveyorBeamBreak.get();
  }

  public boolean isInIntake() {
    return !intakeBeamBreak.get();
  }

  public void setSpeedConveyor(double speed) {
    mConveyorMotor.set(speed);
  }

  @Override
  public void periodic() {
    if (Constants.FMSDETACHED) {
      SmartDashboard.putBoolean("Intake Beam Break", isInIntake());
      SmartDashboard.putNumber("OnFieldIntakeCheck", isInIntake() ? 1 : 0);
      SmartDashboard.putNumber("OnFieldConveyerCheck", isInConveyor() ? 1 : 0);
      SmartDashboard.putBoolean("WE GOT IT (Conveyer Beam Break)", isInConveyor());
      SmartDashboard.putBoolean("Intake Solonoid State", solonoidState);
      // SmartDashboard.putNumber("Conveyor Speed",
      // mConveyorMotor.getEncoder().getVelocity());
      // SmartDashboard.putNumber("Uptake Speed",
      // mUptakeMotor.getEncoder().getPosition());
      // SmartDashboard.putNumber("OTB Intake",
      // mBeltMotor.getEncoder().getVelocity());
      // SmartDashboard.putBoolean("Amp Bar Deployed", isDeployed());
      // SmartDashboard.putNumber("conveyer current",
      // mConveyorMotor.getOutputCurrent());

      Logger.recordOutput("WE GOT IT (Conveyer Beam Break)", isInConveyor());
      Logger.recordOutput("Intake Beam Break", isInIntake());
    }

  }

  // This method will be called once per scheduler run

}
