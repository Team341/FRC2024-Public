// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.DaisyMath;

public class ShooterHood extends SubsystemBase {
  /** Creates a new ShooterHood. */
  private TreeMap<Double, Double> mAngleTable = new TreeMap<>();

  public static ShooterHood instance = null;
  // public static SparkAbsoluteEncoder mAbsoluteEncoder;
  private static CANcoder mHoodEncoder;
  private CANSparkMax mHoodMotor;
  public static SparkPIDController mHoodController;
  private double mAlphaPosition = 0.0;
  private CANcoderConfiguration CANCoderConfig;
  public static PIDController hoodPID;
  private MagnetSensorConfigs magConfigs;
  private SimpleMotorFeedforward hoodFF;
  private double encoderRelativeOffset;
  public boolean isRed;
  double error = 100;
  public PIDController absolutePID;

  public static ShooterHood getInstance() {
    if (instance != null) {
      return instance;
    }
    instance = new ShooterHood();
    return instance;
  }

  public ShooterHood() {
    SmartDashboard.putNumber("Hood Offset ReturnVal", 0);
    absolutePID = new PIDController(5. / 90., 0., 1. / 1000.);

    
    mAngleTable.put(0., 11.);
    mAngleTable.put(1.0, 11.);
    mAngleTable.put(1.4, 1.);

    mAngleTable.put(1.7, 0.);
    mAngleTable.put(1.83, 0.5);

    mAngleTable.put(2.1, 0.);
    mAngleTable.put(2.2, .5);

    mAngleTable.put(2.4, 1.);
    mAngleTable.put(2.8, .5);
    mAngleTable.put(3.0, .5);

    mAngleTable.put(3.2, .5);
    mAngleTable.put(3.5, .5);
    mAngleTable.put(3.7, .25);
    mAngleTable.put(300.5, .25);

    
    CANCoderConfig = new CANcoderConfiguration();
    magConfigs = new MagnetSensorConfigs();
    magConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    mHoodMotor = new CANSparkMax(Constants.ShooterHood.HOOD_MOTOR_PORT, MotorType.kBrushless);
    mHoodMotor.setInverted(true);
    mHoodMotor.setIdleMode(IdleMode.kBrake);
    mHoodEncoder = new CANcoder(Constants.ShooterHood.ENCODER_PORT, "CANIVORE 3");
    mHoodMotor.setSoftLimit(SoftLimitDirection.kForward, (float) (75. / 360.));
    mHoodMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) (-5. / 360.));
    mHoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mHoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    hoodPID = new PIDController(Constants.ShooterHood.PIDs.kP, Constants.ShooterHood.PIDs.kI,
        Constants.ShooterHood.PIDs.kD);
    hoodFF = new SimpleMotorFeedforward(Constants.ShooterHood.PIDs.kS, Constants.ShooterHood.PIDs.kV);
    mHoodEncoder.getConfigurator().apply(magConfigs);
    Timer.delay(1.);

    hoodPID.enableContinuousInput(-180., 180.);


    mHoodController = mHoodMotor.getPIDController();

    mHoodController.setP(Constants.ShooterHood.PIDs.kP);
    mHoodController.setI(Constants.ShooterHood.PIDs.kI);
    mHoodController.setD(Constants.ShooterHood.PIDs.kD);
    mHoodController.setFF(Constants.ShooterHood.PIDs.kF);
    mHoodMotor.getEncoder().setPositionConversionFactor(1. / (44. / 16. * 45.));

    mHoodMotor.getEncoder().setPosition(getPositionAbsolute() / 360.);
    mHoodMotor.burnFlash();
   

    Constants.ShooterHood.PIDs.logToDashboard();
  }

  public void resetMotorPos() {
    mHoodMotor.getEncoder().setPosition(getPositionAbsolute() / 360.);

  }

  public double getSpeed() {
    return mHoodMotor.getAppliedOutput();
  }

  public void setSpeed(double speed) {
    mHoodMotor.set(speed);
  }

  public void setEncoderPosition(double positionDegrees) {
    encoderRelativeOffset = -positionDegrees - mHoodEncoder.getPosition().getValueAsDouble();
  }

  public void setPositionAbsolute(double target) {
    error = target - getPositionAbsolute();
    mHoodMotor.set(absolutePID.calculate(getPositionAbsolute(), target));

  }

  // feedforward?

  public void setPosition(double target) {
    error = target - mHoodMotor.getEncoder().getPosition() * 360.;

    mHoodController.setReference(target / 360., ControlType.kPosition);
  
  }

  public double getPositionAbsolute() {
    return Constants.realRobot
        ? -((mHoodEncoder.getAbsolutePosition().getValueAsDouble() + Constants.ShooterHood.ABSOLUTE_ENCODER_OFFSET)
            * 360) / 2.75
        : ((mHoodEncoder.getAbsolutePosition().getValueAsDouble() + Constants.ShooterHood.ABSOLUTE_ENCODER_OFFSET)
            * 360) / 2.75; // 0.448

   
  }

  public boolean atPosition() {
    return Math.abs(error) < 0.25;
  }

  public boolean atPosition(double tolerance) {
    return Math.abs(error) < tolerance;
  }

  public double getAbsolutePositionRaw() {
    return mHoodEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public Translation3d getLimeLightPosition() {
    double dx = Constants.Shooter.SHOOTER_LENGTH * Math.cos(Math.toRadians(90. - getPositionAbsolute()));
    double dz = Constants.Shooter.SHOOTER_LENGTH * Math.sin(Math.toRadians(90. - getPositionAbsolute()));

    return (new Translation3d(Constants.Shooter.PIVOT_SHOOTER_POSITION.getX() - dx, 0.,
        dz + Constants.Shooter.PIVOT_SHOOTER_POSITION.getZ()));
  }

  public void setPositionPivot(DoubleSupplier goal, double alpha) {
 
    error = goal.getAsDouble() - mHoodMotor.getEncoder().getPosition() * 360.;
    mAlphaPosition = alpha * goal.getAsDouble() + (1. - alpha) * mAlphaPosition;
    setPosition(mAlphaPosition);

  }

  public double getDistanceToSpeaker() {
    var pose = Swerve.getInstance().getPose();
 
    double dx = Math.abs(pose.getX() - Constants.FieldAutoPoses.speakerPose.getX());
    double dy = Math.abs(pose.getY() - Constants.FieldAutoPoses.speakerPose.getY());
    double shooterX = Constants.Shooter.SHOOTER_X_OFFSET;
    double distance = (Math.sqrt(dy * dy + Math.pow(dx - shooterX, 2)));
    return distance;
  }

  public double getDistanceToSpeakerFuture(double time) {
    var pose = Swerve.getInstance().getPoseVisionOdom();
    ChassisSpeeds newSpeeds = Swerve.getInstance().getRelativeSpeeds();
    SmartDashboard.putNumber("newSpeeds X", newSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("newSpeeds Y", newSpeeds.vyMetersPerSecond);

    if (Swerve.getInstance().isRed) {
      pose = new Pose2d(new Translation2d((pose.getX() + -newSpeeds.vxMetersPerSecond * time),
          (pose.getY() + -newSpeeds.vyMetersPerSecond * time)), pose.getRotation());
    } else {
      pose = new Pose2d(new Translation2d((pose.getX() + newSpeeds.vxMetersPerSecond * time),
          (pose.getY() + newSpeeds.vyMetersPerSecond * time)), pose.getRotation());
    }

    double dx = Math.abs(pose.getX() - Constants.FieldAutoPoses.speakerPose.getX());

    double dy = Math.abs(pose.getY() - Constants.FieldAutoPoses.speakerPose.getY());
    double shooterX = Constants.Shooter.SHOOTER_X_OFFSET;
    double distance = (Math.sqrt(dy * dy + Math.pow(dx - shooterX, 2)));
    return distance;
  }

  public double getShooterSpeedRight() {
    if (getDistanceToSpeaker() >= 5.0)
      return 1.;
    return 0.6;
  }

  public double getAngleFromFutureRange() {

    double h = Constants.FieldConstants.SPEAKER_HEIGHT;
    double heightOfShooter = Constants.Shooter.SHOOTER_HEIGHT;

    double lowKey = -1.0;
    double lowVal = -1.0;
    double returnVal = 2.;
    double distance = getDistanceToSpeakerFuture(0.0);

    for (double key : mAngleTable.keySet()) {
      if (distance < key) {
        double highVal = mAngleTable.get(key);
        if (lowKey >= 0.0) {
          double m = (highVal - lowVal) / (key - lowKey);
          returnVal = lowVal + m * (distance - lowKey);
          break;
        } else {
          
        }
      }
      lowKey = key;
      lowVal = mAngleTable.get(key);
    }

    if (Constants.FMSDETACHED) {
      SmartDashboard.putNumber("hood return val", returnVal);
      SmartDashboard.putNumber("actual distance lmao", distance);
    }
    // get returnval eventually
    Logger.recordOutput("Distance to Goal", distance);

    double theta = Math.toDegrees(Math.atan((h - heightOfShooter) / (distance)))
        + returnVal;

    if (!Swerve.getInstance().isOdometeryGood) {
      theta += 2.0;
    }
    Logger.recordOutput("Goal Hood Angle", theta);
    return DaisyMath.minmax(theta, 0., 70.);

  }

  @Override
  public void periodic() {

    if (Constants.FMSDETACHED) {
      SmartDashboard.putNumber("Hood Velocity",
      mHoodMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("Hood Angle Absolute", getPositionAbsolute());
      SmartDashboard.putNumber("Hood absolute voltage output", getSpeed());
      SmartDashboard.putNumber("Hood Position Raw", getAbsolutePositionRaw());
      SmartDashboard.putNumber("Hood Position Motor",
          mHoodMotor.getEncoder().getPosition() * 360);
      SmartDashboard.putBoolean("Hood At Position",
          atPosition());
      SmartDashboard.putNumber("Future Hoodangle", getAngleFromFutureRange());
    }
    Logger.recordOutput("Hood Angle Absolute", getPositionAbsolute());
    Logger.recordOutput("shooter hood error", error);

  }
}
