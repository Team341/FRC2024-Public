// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
//Revup that does not set hood
public class RevUpTeleop extends Command {
  /** Creates a new RevUp. */
  Shooter shoot;
  double mLeftSpeed;
  DoubleSupplier mRightSpeed;
  Intake mIntake;

  public RevUpTeleop(Shooter shoot, double leftSpeed, DoubleSupplier rightSpeed) {
    this.shoot = shoot;
    mLeftSpeed = leftSpeed;
    mRightSpeed = rightSpeed;
    addRequirements(shoot);
    // Use addRequirements() here to decl+50are subsystem dependencies.
  }

  int cnt = 0;
  boolean found = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.Shooter.SHOOTER_GAINS.updateFromDashboard();
    cnt = 0;
    found = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.setSpeed(mLeftSpeed * Constants.Shooter.MAX_SHOOTER_SPEED_RPS,
        mRightSpeed.getAsDouble() * Constants.Shooter.MAX_SHOOTER_SPEED_RPS);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.setPercentOutput(0., 0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
