package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utilities.DaisyMath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveDrivePoseEstimator visionOdometry;

    public MK4iSwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private ProfiledPIDController mThetaController;

    private Rotation2d headingSetpoint = new Rotation2d();
    private boolean isLockedHeadingMode = false;

    private static Swerve instance;
    public boolean isTeleop = false;
    public boolean isEnabled = false;
    public boolean isOdometeryGood = false;
    public boolean inBox = false;
    public int numEstimates = 0;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANIVORE_NAME);
        zeroGyro();

        mSwerveMods = new MK4iSwerveModule[] {
                new MK4iSwerveModule(0, Constants.Swerve.Mod0.constants),
                new MK4iSwerveModule(1, Constants.Swerve.Mod1.constants),
                new MK4iSwerveModule(2, Constants.Swerve.Mod2.constants),
                new MK4iSwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        mThetaController = new ProfiledPIDController(
                Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
                Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
                Constants.Drivebase.THETA_CONTROLLER_GAINS.kD,
                Constants.Drivebase.kThetaControllerConstraints);

        mThetaController.enableContinuousInput(-Math.PI, Math.PI);
        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.3, 0.3, 0.3));

        visionOdometry = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.3, 0.3, 0.3));

        resetOdometry(getPose());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRelativeSpeeds,
                this::driveFromChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(Constants.Drivebase.AUTO_PID_GAINS.kP, Constants.Drivebase.AUTO_PID_GAINS.kI,
                                Constants.Drivebase.AUTO_PID_GAINS.kD),
                        new PIDConstants(Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
                                Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
                                Constants.Drivebase.THETA_CONTROLLER_GAINS.kD),
                        Constants.Swerve.maxSpeed,
                        Units.inchesToMeters(17.14666),
                        new ReplanningConfig()),
                () -> isFlipped(), this);

    }

    public boolean isRed;

    public void setPathSide(boolean isBlue) {
        isRed = !isBlue;
    }

    public boolean isFlipped() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public Command followPathCommand(String pathName, boolean firstPath) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        PathPlannerPath path2;

        if (isRed)
            path2 = path.flipPath();
        else
            path2 = path;

        if (firstPath) {
            return Commands.runOnce(() -> resetOdometry(path2.getPreviewStartingHolonomicPose()), this)
                    .andThen(new FollowPathHolonomic(
                            path,
                            this::getPose,
                            this::getRelativeSpeeds,
                            this::driveFromChassisSpeeds,
                            new HolonomicPathFollowerConfig(
                                    new PIDConstants(Constants.Drivebase.AUTO_PID_GAINS.kP,
                                            Constants.Drivebase.AUTO_PID_GAINS.kI,
                                            Constants.Drivebase.AUTO_PID_GAINS.kD),
                                    new PIDConstants(Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
                                            Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
                                            Constants.Drivebase.THETA_CONTROLLER_GAINS.kD),
                                    Constants.Swerve.maxSpeed,
                                    Units.inchesToMeters(17.14666),
                                    new ReplanningConfig()),
                            () -> {
                               
                                return isRed;
                            }, this))
                    .andThen(new InstantCommand(() -> drive(new Translation2d(), 0, true, false, 0), this));
        } else {
            return new FollowPathHolonomic(
                    path,
                    this::getPose,
                    this::getRelativeSpeeds,
                    this::driveFromChassisSpeeds,
                    new HolonomicPathFollowerConfig(
                            new PIDConstants(Constants.Drivebase.AUTO_PID_GAINS.kP,
                                    Constants.Drivebase.AUTO_PID_GAINS.kI,
                                    Constants.Drivebase.AUTO_PID_GAINS.kD),
                            new PIDConstants(Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
                                    Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
                                    Constants.Drivebase.THETA_CONTROLLER_GAINS.kD),
                            Constants.Swerve.maxSpeed,
                            Units.inchesToMeters(17.14666),
                            new ReplanningConfig()),
                    () -> {
                   
                        return isRed;
                    }, this).andThen(new InstantCommand(() -> drive(new Translation2d(), 0, true, false, 0), this));
        }
    }

    public Command followChoreoPath(String pathName, boolean firstPath) {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
        PathPlannerPath path2;

  
        if (isRed)
            path2 = path.flipPath();
        else
            path2 = path;

        if (firstPath) {
            return Commands.runOnce(() -> resetOdometry(path2.getPreviewStartingHolonomicPose()), this)
                    .andThen(new FollowPathHolonomic(
                            path,
                            this::getPose,
                            this::getRelativeSpeeds,
                            this::driveFromChassisSpeeds,
                            new HolonomicPathFollowerConfig(
                                    new PIDConstants(Constants.Drivebase.AUTO_PID_GAINS.kP,
                                            Constants.Drivebase.AUTO_PID_GAINS.kI,
                                            Constants.Drivebase.AUTO_PID_GAINS.kD),
                                    new PIDConstants(Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
                                            Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
                                            Constants.Drivebase.THETA_CONTROLLER_GAINS.kD),
                                    Constants.Swerve.maxSpeed,
                                    Units.inchesToMeters(17.14666),
                                    new ReplanningConfig()),
                            () -> {
                                // var alliance2 = DriverStation.getAlliance();
                                // if (alliance2.isPresent()) {
                                // return alliance2.get() == DriverStation.Alliance.Red;
                                // }
                                return isRed;
                                // return false;
                            }, this))
                    .andThen(new InstantCommand(() -> drive(new Translation2d(), 0, true, false, 0), this));
        } else {
            return new FollowPathHolonomic(
                    path,
                    this::getPose,
                    this::getRelativeSpeeds,
                    this::driveFromChassisSpeeds,
                    new HolonomicPathFollowerConfig(
                            new PIDConstants(Constants.Drivebase.AUTO_PID_GAINS.kP,
                                    Constants.Drivebase.AUTO_PID_GAINS.kI,
                                    Constants.Drivebase.AUTO_PID_GAINS.kD),
                            new PIDConstants(Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
                                    Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
                                    Constants.Drivebase.THETA_CONTROLLER_GAINS.kD),
                            Constants.Swerve.maxSpeed,
                            Units.inchesToMeters(17.14666),
                            new ReplanningConfig()),
                    () -> {
                        // var alliance2 = DriverStation.getAlliance();
                        // if (alliance2.isPresent()) {
                        // return alliance2.get() == DriverStation.Alliance.Red;
                        // }
                        // return false;
                        return isRed;
                    }, this).andThen(new InstantCommand(() -> drive(new Translation2d(), 0, true, false, 0), this));
        }
    }

    public void setHeadingGoal(Rotation2d heading) {
        headingSetpoint = heading;
    }

    public void enableLockedHeadingMode(boolean locked) {
        isLockedHeadingMode = locked;
    }

    public Rotation2d getHeadingGoal() {
        return headingSetpoint;
    }

    public boolean isInLockedHeadingMode() {
        return isLockedHeadingMode;
    }

    public void invertTurnModules(boolean isInverted) {
        mSwerveMods[0].setTurnMotorInverted(isInverted);
        mSwerveMods[1].setTurnMotorInverted(isInverted);
        mSwerveMods[2].setTurnMotorInverted(isInverted);
        mSwerveMods[3].setTurnMotorInverted(isInverted);
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch().getValueAsDouble());
    }

    public void setBrakeMode(boolean brakeMode) {
        mSwerveMods[0].setBrakeMode(brakeMode);
        mSwerveMods[1].setBrakeMode(brakeMode);
        mSwerveMods[2].setBrakeMode(brakeMode);
        mSwerveMods[3].setBrakeMode(brakeMode);
    }

    public double getAngularSpeed() {

        return gyro.getAngularVelocityZDevice().getValue();

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            double speed) {
        // SmartDashboard.putNumber("Translation X", translation.getX());
        // SmartDashboard.putNumber("Translation Y", translation.getY());

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speed);

        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates, double speed) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, speed);

        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void driveFromChassisSpeeds(ChassisSpeeds cSpeed) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(cSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }

    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStatesOverride(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (MK4iSwerveModule mod : mSwerveMods) {

            mod.setDesiredStateOverride(desiredStates[mod.moduleNumber], false);

        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public Pose2d getPoseVisionOdom() {
        return visionOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {

        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        visionOdometry.resetPosition(getYaw(), getModulePositions(), pose);

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (MK4iSwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (MK4iSwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void setYaw(double y) {
        gyro.setYaw(y);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble())
                : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /**
     * @return theta controller
     */
    public ProfiledPIDController getThetaController() {
        return mThetaController;
    }

    public void resetModulesToAbsolute() {
        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }

    }

    public Translation3d getNormalVector3d() {
        return new Translation3d(0, 0, 1).rotateBy(new Rotation3d(
                Units.degreesToRadians(gyro.getRoll().getValueAsDouble()),
                Units.degreesToRadians(gyro.getPitch().getValueAsDouble()),
                0));
    }

    public void addVisionPoseEstimate(Pair<Pose2d, Double> measurement) {
        Pose2d temp = new Pose2d(measurement.getFirst().getTranslation(), getYaw());
        swerveOdometry.addVisionMeasurement(temp, measurement.getSecond());
    }

    public void addVisionPoseEstimateVisionOdom(Pair<Pose2d, Double> measurement) {
        Pose2d temp = new Pose2d(measurement.getFirst().getTranslation(), getYaw());
        visionOdometry.addVisionMeasurement(temp, measurement.getSecond());
    }

    public void applyCurrentTele() {
        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.configDriveMotorTele();
        }
    }

    public boolean wheelsAligned(double angle) {
        for (MK4iSwerveModule mod : mSwerveMods) {
            if (Math.abs(DaisyMath.boundAngleNeg180to180Degrees(
                    DaisyMath.boundAngleNeg180to180Degrees(mod.getPosition().angle.getDegrees())
                            - DaisyMath.boundAngleNeg180to180Degrees(angle))) > 5.0) {
                return false;
            }

        }
        return true;
    }

    public ChassisSpeeds getRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(mSwerveMods[0].getState(), mSwerveMods[1].getState(),
                mSwerveMods[2].getState(), mSwerveMods[3].getState());
    }

    @Override
    public void periodic() {

        boolean gotVision = false;
        var pose1 = LimelightInterface.getInstance().getRobotPoseInFieldSpace("limelight-host");
        if (pose1 != null) {
            if (isTeleop)
                addVisionPoseEstimate(pose1);
            addVisionPoseEstimateVisionOdom(pose1);
            gotVision = true;
        }
        if (isTeleop) {
            if (isRed) { // TODO CHECK THIS JAWN
                var temp = getModulePositions();
                for (var x : temp) {
                    x.distanceMeters *= -1.;
                }
                swerveOdometry
                        .update(getYaw(), temp);
                visionOdometry.update(getYaw(), temp);
            } else {
                swerveOdometry
                        .update(getYaw(), getModulePositions());
                visionOdometry
                        .update(getYaw(), getModulePositions());
            }
        }

        var pose2 = LimelightInterface.getInstance().getRobotPoseInFieldSpace("limelight-tag");
        if (pose2 != null) {
            if (isTeleop)
                addVisionPoseEstimate(pose2);
            addVisionPoseEstimateVisionOdom(pose2);
            gotVision = true;

        }

        if (isTeleop) {
            double yThresh = 2.5;
            double curX = getPose().getX();
            double curY = getPose().getY();

            if (isRed) {
                double redThreshX = 13.3;

                if (curX < redThreshX && curY < yThresh) {
                    isOdometeryGood = false;
                    inBox = false;
                    numEstimates = 0;
                } else {
                    inBox = true;
                }

            } else {
                double blueThreshX = 3.3;

                if (curX > blueThreshX && curY < yThresh) {
                    isOdometeryGood = false;
                    inBox = false;
                    numEstimates = 0;
                } else {
                    inBox = true;
                }

            }
            if (inBox && gotVision) {
                numEstimates++;
            }
            if (numEstimates > Constants.Vision.GOOD_ODOMETERY_THRESH) {
                isOdometeryGood = true;
            }
        }

        if (!isTeleop) {
            swerveOdometry.update(getYaw(), getModulePositions());
            if (isRed) { // TODO CHECK THIS JAWN
                var temp = getModulePositions();
                for (var x : temp) {
                    x.distanceMeters *= -1.;
                }

                visionOdometry.update(getYaw(), temp);
            } else {
                visionOdometry
                        .update(getYaw(), getModulePositions());
            }
        }

        if (Constants.FMSDETACHED) {
            Logger.recordOutput("IS FLIPPED", isFlipped());
            Logger.recordOutput("Pitch", getPitch().getDegrees());
            Logger.recordOutput("Yaw", getYaw().getDegrees());
            Logger.recordOutput("yaw speed", getAngularSpeed());
        }
        Logger.recordOutput("Swerve/Pose", getPose());
        Logger.recordOutput("Swerve/Pose Vision Odom", getPoseVisionOdom());

        for (MK4iSwerveModule mod : mSwerveMods) {
            if (Constants.FMSDETACHED) {

                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
                        mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
                        mod.getPosition().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                        mod.getState().speedMetersPerSecond);
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Duty Cycle",
                        mod.getDutyCycle());
            }
        }


    }

    public double getRoll() {

        return gyro.getRoll().getValueAsDouble();
    }
}