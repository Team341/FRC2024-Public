// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimelightInterface;
// import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    if (Constants.FMSDETACHED) {
      SmartDashboard.putNumber("RC Hood Target Angle", 30.);
      SmartDashboard.putNumber("RC Left Shooter Target Speed", 54.);
      SmartDashboard.putNumber("RC Right Shooter Target Speed", 90.);
      SmartDashboard.putNumber("limelight delay", 0.2);
    }

    LED.getInstance().clear();
    LimelightInterface.getInstance().setLimeLightLED(1);
    try {
      m_robotContainer = new RobotContainer();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    Logger.recordMetadata("ProjectName", "2024-Beta"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      // pdp = new PowerDistribution(1, ModuleType.kRev); // Enables power
      // distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save
                                                                                            // outputs to
                                                                                            // a new log
    }

    Logger.start();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Swerve.getInstance().isEnabled = false;
    Swerve.getInstance().isTeleop = false;
    Climber.getInstance().isClimbed = false;
    LED.getInstance().clearAnimation();
    if (!Swerve.getInstance().isRed) {
      LED.getInstance().getLED()
          .animate(new ColorFlowAnimation((int) (Color.kDarkBlue.red * 255.), (int) (Color.kDarkBlue.green * 255.),
              (int) (Color.kDarkBlue.blue * 255.), 0, 0.75, Constants.LED.AMP_LED_END_INDEX, Direction.Forward,
              Constants.LED.AMP_LED_START_INDEX));
    }

    else {
      LED.getInstance().getLED()
          .animate(new ColorFlowAnimation((int) (Color.kDarkRed.red * 255.), (int) (Color.kDarkRed.green * 255.),
              (int) (Color.kDarkRed.blue * 255.), 0, 0.75, Constants.LED.AMP_LED_END_INDEX, Direction.Forward,
              Constants.LED.AMP_LED_START_INDEX));
    }

   
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      Constants.FieldAutoPoses.speakerPose = Constants.FieldAutoPoses.SpeakerPoseRed;
      Constants.FieldAutoPoses.ampAreaPose = Constants.FieldAutoPoses.ampAreaRed;
      Swerve.getInstance().setPathSide(false);
      LimelightInterface.getInstance().setLimelightPipelineIndex(0);
    } else {
      Constants.FieldAutoPoses.speakerPose = Constants.FieldAutoPoses.SpeakerPoseBlue;
      Constants.FieldAutoPoses.ampAreaPose = Constants.FieldAutoPoses.ampAreaBlue;
      Swerve.getInstance().setPathSide(true);
      LimelightInterface.getInstance().setLimelightPipelineIndex(1);

    }
    if (!m_robotContainer.builtChooser)
      m_robotContainer.buildAutoChooser();

  }

  @Override
  public void disabledPeriodic() {

    if (!Swerve.getInstance().isRed) {
      LED.getInstance().getLED()
          .animate(new ColorFlowAnimation((int) (Color.kDarkBlue.red * 255.), (int) (Color.kDarkBlue.green * 255.),
              (int) (Color.kDarkBlue.blue * 255.), 0, 0.75,
              Constants.LED.AMP_LED_END_INDEX, Direction.Forward,
              Constants.LED.AMP_LED_START_INDEX));

    }

    else {

      LED.getInstance().getLED()
          .animate(new ColorFlowAnimation((int) (Color.kDarkRed.red * 255.), (int) (Color.kDarkRed.green * 255.),
              (int) (Color.kDarkRed.blue * 255.), 0, 0.75, Constants.LED.AMP_LED_END_INDEX, Direction.Forward,
              Constants.LED.AMP_LED_START_INDEX));
    }

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    Swerve.getInstance().isEnabled = true;
    Swerve.getInstance().isTeleop = false;

    LED.getInstance().clear();

    Swerve.getInstance().setBrakeMode(true);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    Swerve.getInstance().isEnabled = true;

    Swerve.getInstance().isTeleop = true;

    Swerve.getInstance().applyCurrentTele();
    LED.getInstance().clear();

    Swerve.getInstance().setBrakeMode(true);
    m_robotContainer.isFirstTeleop = true;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
