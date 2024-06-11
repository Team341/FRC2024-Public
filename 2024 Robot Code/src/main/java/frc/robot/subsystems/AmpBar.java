// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpBar extends SubsystemBase {
  /** Creates a new AmpBar. */
  private boolean solonoidState;

  public AmpBar() {
    solonoidState = solenoid.get() == DoubleSolenoid.Value.kReverse;
  }

  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.AmpMechanism.FORWARD_PORT_PCH,
      Constants.AmpMechanism.REVERSE_PORT_PCH);

  private static AmpBar instance;

  public static AmpBar getInstance() {
    if (instance == null) {
      instance = new AmpBar();
    }
    return instance;
  }

  public void deploy() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
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
    if (!solonoidState)
      return true;
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.FMSDETACHED) {
    }
    // SmartDashboard.putBoolean("Dampener Deployed", isDeployed());
    // SmartDashboard.putString("Solonoid Reading",solenoid.get().toString()); }
  }
}
