// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */
  private Swerve s_Swerve;
  double angle;

  PIDController thetaController = new PIDController(Math.toRadians(Constants.Drivebase.THETA_CONTROLLER_GAINS.kP), Constants.Drivebase.THETA_CONTROLLER_GAINS.kI, Constants.Drivebase.THETA_CONTROLLER_GAINS.kD);
  public TurnToAngle(Swerve swerve, double angle) {
    s_Swerve =  swerve;
    this.angle = angle;
    addRequirements(s_Swerve);

    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleSpeed = thetaController.calculate( s_Swerve.getYaw().getDegrees(), angle);
    s_Swerve.drive(new Translation2d(), angleSpeed, true , true, Constants.Swerve.maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(), 0, true , true, Constants.Swerve.maxSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(thetaController.getPositionError())<= 3) {
      return true;
    }
    else
      return false;
  }
}
