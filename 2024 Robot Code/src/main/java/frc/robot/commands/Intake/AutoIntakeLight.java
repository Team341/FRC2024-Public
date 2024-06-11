// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeLight extends Command {
  /** Creates a new TeleIntake. */
  Intake intake;
  double speed;
  double counter;

  public AutoIntakeLight(Intake intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.retract();
       intake.setSpeedUptake(0.5);

    intake.setSpeedConveyor(0.);
  }

  boolean trig = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   intake.setSpeedUptake(0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeedUptake(0);
  

    // intake.retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
  return intake.isInIntake();
  }
}
