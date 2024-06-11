// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RevUpAmp extends Command {
  /** Creates a new RevUp. */
  Shooter shoot;
  double mLeftSpeed;
  double mRightSpeed;
  Intake mIntake;
  DoubleSupplier angle;
  ShooterHood hood;

  public RevUpAmp(Shooter shoot, double leftSpeed, double rightSpeed, DoubleSupplier angle, ShooterHood hood) {
    this.shoot = shoot;
    mLeftSpeed = leftSpeed;
    mRightSpeed = rightSpeed;
    this.hood = hood;
    this.angle = angle;
    addRequirements(shoot, hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.Shooter.SHOOTER_GAINS.updateFromDashboard();

    // shoot.leftShooter.getConfigurator().apply(shoot.shooterConfiguration);
    // shoot.leftShooter.getConfigurator().apply(shoot.shooterConfiguration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.setSpeed(mLeftSpeed * Constants.Shooter.MAX_SHOOTER_SPEED_RPS,
        mRightSpeed * Constants.Shooter.MAX_SHOOTER_SPEED_RPS);
   
    hood.setPositionAbsolute(angle.getAsDouble());
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
