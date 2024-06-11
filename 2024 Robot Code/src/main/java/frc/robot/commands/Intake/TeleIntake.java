// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class TeleIntake extends Command {
  /** Creates a new TeleIntake. */
  Intake intake;
  double speed;
  double counter;
  boolean hasNote;
  int cnt;

  public TeleIntake(Intake intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    hasNote = false;
    cnt = 999999;
  }

  boolean trig = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hasNote && cnt < 20) cnt++;


    if (intake.isInIntake()) {
      if (!hasNote) {
        intake.deploy();
        hasNote = true;
        cnt = 0;
      }

    }
    if (cnt > 3) {
          intake.setSpeedIntake(speed);

    }
    else {
      intake.setSpeedIntake(0);
    }
    if (intake.isInConveyor())
      counter++;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeedIntake(.0);

    // intake.retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    if (speed != -1) {
      if (intake.isInConveyor()) {
        return true;
      }
      return false;
    } else {
      return false;

    }
  }
}
