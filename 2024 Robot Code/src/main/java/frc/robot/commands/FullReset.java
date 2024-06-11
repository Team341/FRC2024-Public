// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterHood;

public class FullReset extends Command {
  /** Creates a new FullReset. */
  ShooterHood hood;
  Intake intake;
  Climber climber;
  AmpBar bar;

  public FullReset(ShooterHood mHood, Climber mClimber, Intake mIntake, AmpBar mAmp) {
    hood = mHood;
    intake = mIntake;
    climber = mClimber;
    bar = mAmp;
    addRequirements(climber, hood, intake, bar);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.deploy();
    bar.retract();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    hood.setPositionPivot(() -> Constants.Shooter.SPEAKER_CLOSE_ANGLE, 0.98);
    climber.setSpeedLeft(-0.25);
    climber.setSpeedRight(-0.25);
    LED.getInstance().clearAnimation();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeedLeft(0);
    climber.setSpeedRight(0);
    LED.getInstance().clearAnimation();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
