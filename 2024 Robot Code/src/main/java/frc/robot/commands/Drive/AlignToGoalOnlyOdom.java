// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToGoalOnlyOdom extends Command {
  Swerve mSwerve;
  private PIDController DriveController = new PIDController(
      5.,
      Constants.Drivebase.AUTO_PID_GAINS.kI,
      Constants.Drivebase.AUTO_PID_GAINS.kD);

  /** Creates a new AlignToGoal. */

  public AlignToGoalOnlyOdom(Swerve swerve) {
    mSwerve = swerve;
    addRequirements(mSwerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationVal = 0;
    counter = 0;
    saw = false;
  }

  boolean saw;
  double counter;
  double rotationVal;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var pose = mSwerve.getPoseVisionOdom();

    double dx = Math.abs(pose.getX() - Constants.FieldAutoPoses.speakerPose.getX());
    double dy = pose.getY() - Constants.FieldAutoPoses.speakerPose.getY();
    double setpoint;
    if (Swerve.getInstance().isRed) {
      setpoint = -Math.atan(dy / dx);

    }
    else {
      setpoint = Math.atan(dy / dx);
    }
    rotationVal = DriveController.calculate(pose.getRotation().getRadians(), setpoint);

    rotationVal = Math.signum(rotationVal) * Math.max(Math.abs(rotationVal), .25);
    if (Math.abs(DriveController.getPositionError()) < Math.toRadians(3.)) {

      counter++;
    } else {
      counter = 0;
    }

    Logger.recordOutput("Align Error", 180. / Math.PI * DriveController.getPositionError());
    mSwerve.drive(
        new Translation2d(0, 0),
        rotationVal,
        false,
        true,
        4.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerve.drive(
        new Translation2d(0, 0),
        0.,
        false,
        true,
        4.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 9;
  }
}
