// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.Animation;
// import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.subsystems.StateMachine.ArmPose;
import frc.robot.utilities.PIDGains;
import frc.robot.utilities.SwerveModuleConstants;
import frc.robot.utilities.COTSFalconSwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in t his class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static CTREConfigs CTRE_CONFIGS = new CTREConfigs();

  public static final boolean FMSDETACHED = !DriverStation.isFMSAttached();
  public static final int PDH_PORT = 1;
  public static final double TRIGGER_THRESHOLD = 0.05;
  public static final double STICK_DEADBAND = 0.1;

  public static final boolean realRobot = true;

  public static final class LED {

    public static final int LED_PORT = 1;
    public static final int CANDLE_START_INDEX = 0;
    public static final int CANDLE_LED_LENGTH = 8;
    public static final int AMP_LED_START_INDEX = 8;
    public static final int AMP_LED_END_INDEX = 50;
    public static final Animation WIN_BM = null;
    public static final int LED_STRING_LENGTH = 51;
  }

  public static final class Trap {

    public static final int FORWARD_CHANNEL = 5;
    public static final int REVERSE_CHANNEL = 10;
    public static final int TRAP_BEAMBREAK_PORT = 4;
    public static final int ROLLER_PORT = 27;
    public static final double INTAKE_SPEED = -0.5;
    public static final double OUTTAKE_SPEED = 0.5;
  }

  public static final class FieldConstants {

    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double SPEAKER_HEIGHT = Units.inchesToMeters((78.25+80.)/2.); 
   
    // SCH Bottom - 78.25 - 83 top
    // measure every time. height of field
    // carpet to centerline of speaker
    // opening
    

  }

  public static class FieldAutoPoses {
    public static final Pose2d SpeakerPoseBlue = new Pose2d(0.22, 5.54, Rotation2d.fromDegrees(0.0));

    public static final Pose2d AmpPoseBlue = new Pose2d(1.79, 7.66, Rotation2d.fromDegrees(-90));
    public static Pose2d speakerPose = new Pose2d(16.3, 5.54, Rotation2d.fromDegrees(180.));

    public static final Pose2d AmpPoseRed = new Pose2d(14.69, 7.2, Rotation2d.fromDegrees(90));

    public static final Pose2d SpeakerPoseRed = new Pose2d(16.3, 5.54, Rotation2d.fromDegrees(180.));

    public static final Pose2d StagePoseRed = new Pose2d(12.43, 5.52, Rotation2d.fromDegrees(-122.));

    public static Pose2d ampAreaPose = null;
    public static final Pose2d ampAreaBlue = new Pose2d(1.79, 7.66, Rotation2d.fromDegrees(0));

    public static final Pose2d ampAreaRed = new Pose2d(14.69, 7.2, Rotation2d.fromDegrees(0));

  }

  public static class Climber {
    public static final int LEFT_MOTOR = 21;
    public static final int RIGHT_MOTOR = 22;
    public static final int LEFT_LIMIT_SWITCH = 1;

    public static final int RIGHT_LIMIT_SWITCH = 2;

    // CLIMBER MOTOR = 50, LEFT BB 1, RIGHT BB 2,
  }

  public static class Drivebase {

    public static final double DRIVEBASE_TRACKWIDTH_METERS = Units.inchesToMeters(24.25);
    public static final double DRIVEBASE_WHEELBASE_METERS = Units.inchesToMeters(24.25);

    public static final PIDGains THETA_CONTROLLER_GAINS = new PIDGains(5., 0.0, 0.);

    public static final PIDGains AUTO_ALIGN_THETA_CONTROLLER_GAINS = new PIDGains(3.5 / 2.5, 0.0, 0.1,
        "Auto Align Theta");
    public static final PIDGains AUTO_ALIGN_TRANSLATE_PIDS = new PIDGains(5., 0, 0.1, "Auto Align Translate");

    public static final double kV = 2.3599 / 12.0;
    public static final double kS = 0.71695 / 12.0;
    public static final double kA = 0.42457 / 12.0;

    // Module Gearing: 6.75:1 (14->50, 27->17, 15->45)
    // Wheel Diameter: 4"
    // C = 2*pi*r
    // inches to meter: 0.0254

    public static final PIDGains AUTO_PID_GAINS = new PIDGains(5., 0, 0.0);// 1.6);//0.075);//0.535
                                                                           // DRIVE_PID_GAINS;//new
                                                                           // PIDGains(41.543 / 4.0, 0.0,
                                                                           // 2.6885);

    // public static final PIDGains AUTO_PID_GAINS = new PIDGains(1.75, 0.0,
    // 0.1);//1.6);//0.075);//0.535 DRIVE_PID_GAINS;//new PIDGains(41.543 / 4.0,
    // 0.0, 2.6885);

    public static final double VISION_ANGLE_TOLERANCE = 3.0;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 4;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI * 8;

    public static final Constraints kThetaControllerConstraints = new Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final Constraints AUTO_ALIGN_THETA_CONSTRAINTS = new Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED / 2.);

    public static final double VISION_KF = ((MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 90.0));// 1.0 / (360.0 /
                                                                                           // MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
  }

  public static class ControllerInputs {

    public static final double DEADBAND = 0.05;

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int PROGRAMMER_CONTROLLER_PORT = 2;
  }

  public static class AmpMechanism {
    public static final double INTAKE_ROLLER_SPEED = -1.0;
    public static final double OUTTAKE_ROLLER_SPEED = -1.0;
    public static final double ABSOLUTE_ENCODER_OFFSET = 57.5 + 180.;
    public static final int AmpBarConstant = 0;
    public static final int FORWARD_PORT_PCH = 7;
    public static final int REVERSE_PORT_PCH = 8;
    // public static final int LEFT_LIMIT_SWITCH_PORT = 0;
    // public static final int RIGHT_LIMIT_SWITCH_PORT = ;

  }

  public static class ShooterHood {
    public static final double ABSOLUTE_ENCODER_OFFSET = -0.6813;// -0.7268 : -.418; ///needs 4 sig figs C:
    public static final int HOOD_MOTOR_PORT = 28;
    public static final double LIMELIGHT_ROTATION_RADIUS = Units.inchesToMeters(5.5);
    public static final PIDGains PIDs = new PIDGains(16., 0, 13., 0.0, 1.88, 0.07, "Shooter Hood PID");
    public static final int ENCODER_PORT = 5;
    public static final double CANCODER_TO_HOOD_RATIO = 2.75;
    public static final double CANCODER_TO_MOTOR = 45.;
    public static final double FLIPPED_OFFSET = 0;
  }

  public static class Shooter {
    public static final int RIGHT_SHOOTER_PORT = 18;
    public static final int LEFT_SHOOTER_PORT = 19;
    public static final double MAX_SHOOTER_SPEED_RPS = 90.;
    public static final double RPS_THRESHHOLD = 3.;

    public static final double SHOOTER_SPEED_AMP_RPS = 0.425 * MAX_SHOOTER_SPEED_RPS;
    public static final double SHOOTER_SPEED_SPEAKER_RPS = 70.;

    public static final PIDGains SHOOTER_GAINS = new PIDGains(0.04, 0.3, 0, 0, 0, 0, "Shooter PID Gains");
    public static final double AMP_SCORE_ANGLE = 54.5 - 2.;
    public static final double SPEAKER_CLOSE_ANGLE = 72.; // -10 for the other three
    public static final double REV_SPEED = 0.9;
    public static final double SPEAKER_WING_SCORE_ANGLE = 9.5;
    public static final double SHOOTER_HEIGHT = Units.inchesToMeters(19.340); // centerline of shot
    public static final double SHOOTER_X_OFFSET = Units.inchesToMeters(10.250); // centerline of shot
    public static final double SHOOTER_LENGTH = 0.;
    public static final Translation3d PIVOT_SHOOTER_POSITION = new Translation3d(0, 0, 0);

  }

  public static class Auto {
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 1.25;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI * 4;
    public static final Constraints kThetaControllerAutoConstraints = new Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

  }

  public static class Vision {
    public static final PIDGains VISION_PID_GAINS = new PIDGains(0.04, 0.0, 0.005);

    public static final PIDGains DRIVE_VISION_PID_GAINS = new PIDGains(5.0 / 50.0);
    public static final String LIMELIGHT_NAME = "limelight-host";
    public static final int TAPE_PIPELINE = 1;
    public static final int TAG_PIPELINE = 0;

    public static final double LIMELIGHT_HEIGHT = 0.56;
    public static final Rotation2d HOOD_LIMELIGHT_ANGLE_OFFSET = Rotation2d.fromDegrees(8.);

    public static final int GOOD_ODOMETERY_THRESH = 10; // number of vision estimates added before we trust our
                                                        // odometery for hood tracking

  }

  public static final String CANIVORE_NAME = "CANIVORE 3";

  public static final class Swerve {
    public static final double lengthWithBumpers = Units.inchesToMeters(29.0);
    public static final int pigeonID = 0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
        .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(24.25);
    public static final double wheelBase = Units.inchesToMeters(24.25);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 200.0 / 1000.;
    public static final double closedLoopRamp = 100.0 / 1000.;

    /* Angle Motor PID Values */
   
    public static final PIDGains anglePIDs = new PIDGains(10., 0, 0.4, 0.0, 0.0, 0, "Swerve Angle PIDs");

    /* Drive Motor PID Values */
   
    public static final PIDGains drivePIDs = new PIDGains(.15, 0, 0., 0.0, 0, 0, "Swerve Drive PIDs");

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    public static final double driveKS = 1.0 / 12.0;
    public static final double driveKV = 2.45 / 12.0;
    public static final double driveKA = 0.42457 / 12.0;
    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot

    public static final double SLOW_SPEED_MODIFIER = 0.5; // quarter speed
    public static final double SLOW_ROT_SPEED_MODIFIER = 0.5; // quarter speed

    /** Radians per Second */
    public static final double maxAngularVelocity = 25.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final double FORWARD_PIVOT_POINT = Units.inchesToMeters(40.0);
    public static final double VISION_KF = 0;
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    // 17.14666 inches
    // ALL MODULES ARE RELATIVE TO SHOOTER SIDE FORWARD: INTAKE IS TYPICALLY FORWARD
    // - DON'T WORRY ABOUT IT
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 60;
      public static final int angleMotorID = 14;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(realRobot ? 4.1 : 156.797);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 61;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 9;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(realRobot ? 83.5 : -96.768);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 7;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(realRobot ? 88.9 : 149.238);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 62;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(realRobot ? -97.99 : 32.871);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class Intake {
    public static final int INTAKE_MOTOR_PORT = 17;
    public static final int SOLENOID_PORT = 9;
    public static final int CONVEYOR_MOTOR_PORT = 26;
    public static final int INTAKE_BEAM_BREAK_PORT = 3; // 1000 on real robot
    public static final int CONVEYOR_BEAM_BREAK_PORT = 0;

    public static final double INTAKE_SPEED = 1.;
    public static final double OUTTAKE_SPEED = -1.;
    public static final int BELT_MOTOR_PORT = 25;
    public static final int SOLENOID_PORT_OUT = 6;
    public static final double BELT_MOTOR_SPEED = 0;
  }
}
