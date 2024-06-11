// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbUp extends Command {
  /** Creates a new Climb. */

  Climber mClimber;

  double startPositionRight;
  double startPositionLeft;
  boolean hasLimitSwitchesPressed;

  public ClimbUp(Climber climber) {
    mClimber = climber;
    addRequirements(climber);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasLimitSwitchesPressed = false;
    mClimber.isClimbed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mClimber.getLeftLimit() && mClimber.getRightLimit())
      hasLimitSwitchesPressed = true;
    if(mClimber.beenZeroed) {
      hasLimitSwitchesPressed = true;
    }
    if (!hasLimitSwitchesPressed) {
      if (mClimber.getPositionLeft() > 20) {
        mClimber.setSpeedLeft(-1.);
        
      } else {
        mClimber.setSpeedLeft(-0.25);
        
      }
       if (mClimber.getPositionRight() > 20) {
        mClimber.setSpeedRight(-1.);
        
      } else {
        mClimber.setSpeedRight(-0.25);
        
      }

      return;
    }

    mClimber.setPosition(125.);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    mClimber.setSpeedLeft(0.);
    mClimber.setSpeedRight(0.);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mClimber.atPosition();
  }
}
