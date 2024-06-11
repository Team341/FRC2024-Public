// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class ShootFalconInstant extends Command {
  /** Creates a new ShootFalcon. */
  Shooter shoot;
  ShooterHood hood;
  double mLeftSpeed;
  double mRightSpeed;
  Intake mIntake;
  DoubleSupplier targetPos;
  double pos;
  Boolean threshold;
  int counter;
  int counter2 = 0;
  double start;

  public ShootFalconInstant(Shooter shoot, double leftSpeed, double rightSpeed, Intake intake, ShooterHood hood,
      DoubleSupplier targetPos) {
    this.shoot = shoot;
    this.hood = hood;

    mLeftSpeed = leftSpeed;
    mRightSpeed = rightSpeed;
    mIntake = intake;
    this.targetPos = targetPos;

    addRequirements(shoot, mIntake, hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter2 = 0;
    counter = 0;
   
    threshold = false;
    start = Timer.getFPGATimestamp();
    pos = targetPos.getAsDouble();

    counter2 = 0;
    LED.getInstance().isShooting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {


    shoot.setSpeed(mLeftSpeed*Constants.Shooter.MAX_SHOOTER_SPEED_RPS, mRightSpeed*Constants.Shooter.MAX_SHOOTER_SPEED_RPS);
    hood.setPositionAbsolute(pos);

    if (shoot.atSpeed() && hood.atPosition())
        counter++;
      else
        counter = 0;

    if (counter > 10 || Timer.getFPGATimestamp() - start >= 0.5) {
      mIntake.setSpeedConveyor(1.);
    }
    if (!mIntake.isInConveyor()) {
      counter2++;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shoot.setPercentOutput(0., 0.);
    mIntake.setSpeedConveyor(0.);
    mIntake.setSpeedIntake(0);
    LED.getInstance().isShooting = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter2 > 15;
  }
}
