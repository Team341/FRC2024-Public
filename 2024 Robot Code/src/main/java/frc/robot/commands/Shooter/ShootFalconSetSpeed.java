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

//Shoot Falcon w/Setspeed
public class ShootFalconSetSpeed extends Command {
  /** Creates a new ShootFalcon. */
  Shooter shoot;
  ShooterHood hood;
  DoubleSupplier mLeftSpeed;
  DoubleSupplier mRightSpeed;
  Intake mIntake;
  DoubleSupplier targetPos;
  Boolean threshold;
  int counter;
  int counter2 = 0;
  double start;

  public ShootFalconSetSpeed(Shooter shoot, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, Intake intake, ShooterHood hood,
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
    // System.out.println("KPPPPP" + shoot.shooterConfiguration.Slot0.kP);
    Constants.Shooter.SHOOTER_GAINS.updateFromDashboard();
    shoot.shooterConfigurationLeft.Slot0.kP = Constants.Shooter.SHOOTER_GAINS.kP;
    shoot.shooterConfigurationLeft.Slot0.kI = Constants.Shooter.SHOOTER_GAINS.kI;
    shoot.shooterConfigurationLeft.Slot0.kD = Constants.Shooter.SHOOTER_GAINS.kD;

    shoot.shooterConfigurationRight.Slot0.kP = Constants.Shooter.SHOOTER_GAINS.kP;
    shoot.shooterConfigurationRight.Slot0.kI = Constants.Shooter.SHOOTER_GAINS.kI;
    shoot.shooterConfigurationRight.Slot0.kD = Constants.Shooter.SHOOTER_GAINS.kD;
    threshold = false;
    start = Timer.getFPGATimestamp();
    shoot.leftShooter.getConfigurator().apply(shoot.shooterConfigurationLeft);
    shoot.rightShooter.getConfigurator().apply(shoot.shooterConfigurationRight);

    counter2 = 0;
    LED.getInstance().isShooting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // targetPos = SmartDashboard.getNumber("RC Hood Target Angle", 20.);

    // shoot.setSpeed(mLeftSpeed, mRightSpeed);
    shoot.setSpeed(mLeftSpeed.getAsDouble(),
        mRightSpeed.getAsDouble());
    hood.setPositionAbsolute(targetPos.getAsDouble());
    // shoot.setSpeed(mLeftSpeed, mRightSpeed);

    if (shoot.atSpeed() && hood.atPosition())
      counter++;
    else
      counter = 0;

    if (counter > 10) {
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
