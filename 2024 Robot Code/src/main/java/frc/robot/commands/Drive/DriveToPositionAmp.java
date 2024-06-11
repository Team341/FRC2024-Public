package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;
// import frc.robot.subsystems.LimelightInterface;
import java.util.function.Supplier;

public class DriveToPositionAmp extends Command {
    private ProfiledPIDController mThetaController = new ProfiledPIDController(
            Constants.Drivebase.AUTO_ALIGN_THETA_CONTROLLER_GAINS.kP,
            Constants.Drivebase.AUTO_ALIGN_THETA_CONTROLLER_GAINS.kI,
            Constants.Drivebase.AUTO_ALIGN_THETA_CONTROLLER_GAINS.kD,
            Constants.Drivebase.AUTO_ALIGN_THETA_CONSTRAINTS);

    private final Swerve swerveDriveSubsystem;

    private Supplier<Pose2d> targetPoseSupplier;
    private final ProfiledPIDController xController = new ProfiledPIDController(
            Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kP,
            Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kI,
            Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kD,
            new Constraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed * 1.5));

    private final ProfiledPIDController yController = new ProfiledPIDController(
            Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kP,
            Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kI,
            Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kD,
            new Constraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed * 1.5));
    DoubleSupplier ySpeed;

    /**
     * Drives to the given pose on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently
     * assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public DriveToPositionAmp(Swerve swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier, DoubleSupplier yspeed) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        mThetaController.setTolerance(Math.toRadians(0.25));
        mThetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.ySpeed = yspeed;
        addRequirements(swerveDriveSubsystem);
    }

    Supplier<Transform2d> mTargetTransformSupplier;

    double counter;

    @Override
    public void initialize() {
        counter = 0;
        xController.reset(swerveDriveSubsystem.getPose().getX());
        yController.reset(swerveDriveSubsystem.getPose().getY());

        xController.setTolerance(Units.inchesToMeters(1.25));
        yController.setTolerance(Units.inchesToMeters(1.25));

        // LimelightInterface.isAligned = false;
        Constants.Drivebase.AUTO_ALIGN_THETA_CONTROLLER_GAINS.logToDashboard();
        Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.logToDashboard();

        targetPoseSupplier = () -> (!DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                                ? Constants.FieldAutoPoses.AmpPoseRed
                                : Constants.FieldAutoPoses.AmpPoseBlue); 

    }

    @Override
    public void execute() {
        Constants.Drivebase.AUTO_ALIGN_THETA_CONTROLLER_GAINS.updateFromDashboard();
        Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.updateFromDashboard();

        xController.setP(Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kP);
        xController.setI(Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kI);
        xController.setD(Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kD);

        yController.setP(Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kP);
        yController.setI(Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kI);
        yController.setD(Constants.Drivebase.AUTO_ALIGN_TRANSLATE_PIDS.kD);

        mThetaController.setP(Constants.Drivebase.AUTO_ALIGN_THETA_CONTROLLER_GAINS.kP);
        mThetaController.setI(Constants.Drivebase.AUTO_ALIGN_THETA_CONTROLLER_GAINS.kI);
        mThetaController.setD(Constants.Drivebase.AUTO_ALIGN_THETA_CONTROLLER_GAINS.kD);

        Pose2d robotPose = swerveDriveSubsystem.getPose();
        Pose2d targetPose = targetPoseSupplier.get();

        if (targetPose == null) {
            return;
        }

        // if (Math.abs(robotPose.getX() - targetPose.getX()) <=
        // Units.inchesToMeters(1.)
        // && Math.abs(robotPose.getY() - targetPose.getY()) <=
        // Units.inchesToMeters(1.)) {
        // counter++;
        // if (counter > 10) {
        // LimelightInterface.stopUpdating = true;
        // }
        // } else {
        // counter = 0;
        // }

        if (Constants.FMSDETACHED) {

            SmartDashboard.putString("targetPose", targetPose.toString());
            SmartDashboard.putString("robotPose", robotPose.toString());

        }

        xController.setGoal(targetPose.getX());
        mThetaController.setGoal(targetPose.getRotation().getRadians());

        // Drive to the target
        var xSpeed = xController.calculate(robotPose.getX());

        var thetaSpeed = mThetaController.calculate(robotPose.getRotation().getRadians());

        if (xController.atSetpoint()) {
            xSpeed = 0;

        }

        if (mThetaController.atSetpoint()) {
            thetaSpeed = 0;

        }

        swerveDriveSubsystem.drive(new Translation2d(xSpeed, ySpeed.getAsDouble()), thetaSpeed, true, false,
                Constants.Swerve.maxSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        // LimelightInterface.isAligned = true;
        // LimelightInterface.stopUpdating = false;

        swerveDriveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false, false, Constants.Swerve.maxSpeed);
    }
}