package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.Swerve;
import frc.robot.utilities.SlewRateLimiter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier useSlowSpeed;
    private BooleanSupplier superSlowMode;
    private BooleanSupplier useAlign;
    private BooleanSupplier useTurn90;
    private BooleanSupplier useTurnSource;
    private BooleanSupplier leftTrigger;
    private BooleanSupplier useAmpAreaAim;

    private SlewRateLimiter filterDrive = new SlewRateLimiter(32.0 * 4);
    private SlewRateLimiter filterStrafe = new SlewRateLimiter(32.0 * 4);
    private PIDController DriveController = new PIDController(
            5.,
            Constants.Drivebase.AUTO_PID_GAINS.kI,
            Constants.Drivebase.AUTO_PID_GAINS.kD);

    private boolean saw = false;
    private double lastAngle = 0.0;
    private double lastTime = 999999.;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup,
            BooleanSupplier useSlowSpeed, BooleanSupplier superSlowMode, BooleanSupplier useAlign,
            BooleanSupplier turnTo90, BooleanSupplier useTurnSource,
            BooleanSupplier rightTrigger, BooleanSupplier useAmpAreaAim) {
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.useSlowSpeed = useSlowSpeed;
        this.useAlign = useAlign;
        useTurn90 = turnTo90;
        this.useTurnSource = useTurnSource;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.leftTrigger = rightTrigger;
        this.useAmpAreaAim = useAmpAreaAim;
        this.superSlowMode = superSlowMode;
        DriveController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(s_Swerve);

    }

    @Override
    public void initialize() {
        lastAngle = 0.0;
        SmartDashboard.putNumber("Signum", 0.02);
        lastTime = 999999.;

    }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double maxSpeed = Constants.Swerve.maxSpeed;
        double maxRot = Constants.Swerve.maxAngularVelocity;

        if (useSlowSpeed.getAsBoolean()) {
            // The driver is holding a button to reduce the speed
            maxSpeed *= Constants.Swerve.SLOW_SPEED_MODIFIER;
            maxRot *= Constants.Swerve.SLOW_ROT_SPEED_MODIFIER;
        } else if (superSlowMode.getAsBoolean()) {
            // The driver is holding a button to reduce the speed
            maxSpeed *= Constants.Swerve.SLOW_SPEED_MODIFIER / 2.;
            maxRot *= Constants.Swerve.SLOW_ROT_SPEED_MODIFIER / 2.;
        }

        double translationVal = filterDrive.calculate(
                Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND), 1) * maxSpeed);
        double strafeVal = filterStrafe.calculate(
                Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND), 1) * maxSpeed);
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND), 3)
                * maxRot;
        if (Math.abs(rotationVal) > 0.) {
            lastTime = Timer.getFPGATimestamp();
            lastAngle = -99;
        }

        if (useTurn90.getAsBoolean()) {
            if (Swerve.getInstance().isRed) {
                rotationVal = DriveController.calculate(s_Swerve.getYaw().getRadians(), Math.PI / 2);
                lastAngle = Math.PI / 2;
            } else {
                rotationVal = DriveController.calculate(s_Swerve.getYaw().getRadians(), -Math.PI / 2);
                lastAngle = -Math.PI / 2;
            }
            lastTime = Timer.getFPGATimestamp();

        }

        else if (useTurnSource.getAsBoolean()) {
            if (Swerve.getInstance().isRed) {
                rotationVal = DriveController.calculate(s_Swerve.getYaw().getRadians(), Math.PI / 6);
                lastAngle = Math.PI / 6;
            } else {
                rotationVal = DriveController.calculate(s_Swerve.getYaw().getRadians(), -Math.PI / 6);
                lastAngle = -Math.PI / 6;
            }
            lastTime = Timer.getFPGATimestamp();
        } else if (useAmpAreaAim.getAsBoolean()) {
            if (Swerve.getInstance().isRed) {
                rotationVal = DriveController.calculate(s_Swerve.getYaw().getRadians(), Math.PI / 180. * 42);
                lastAngle = Math.PI / 180. * 36;
            } else {
                rotationVal = DriveController.calculate(s_Swerve.getYaw().getRadians(), -Math.PI / 180. * 42);
                lastAngle = -Math.PI / 180. * 36;
            }
            lastTime = Timer.getFPGATimestamp();

        } else if (Math.abs(rotationVal) <= 0. && Timer.getFPGATimestamp() - lastTime > 0.5) {
            if (lastAngle == -99)
                lastAngle = s_Swerve.getYaw().getRadians();
            rotationVal = DriveController.calculate(s_Swerve.getYaw().getRadians(),
                    lastAngle);

        }
        if (useAlign.getAsBoolean()) {

            if (saw || (LimelightInterface.getInstance().hasTarget("limelight-host")
                    && (LimelightInterface.getInstance().getID("limelight-host") == 4
                            || LimelightInterface.getInstance().getID("limelight-host") == 7))) {
       
                rotationVal = -DriveController.calculate(LimelightInterface.getInstance().tx * Math.PI / 180., 0.);
                // We are trying to hold the swerve in a specific direction, use the PID
                // controller to do it
                saw = true;
            } else {
                var pose = s_Swerve.getPose();
                double setpoint;
                double dx = Math.abs(pose.getX() - Constants.FieldAutoPoses.speakerPose.getX());
                double dy = pose.getY() - Constants.FieldAutoPoses.speakerPose.getY();
                if (Swerve.getInstance().isRed) {
                    setpoint = -Math.atan(dy / dx);
              
                  }
                  else {
                    setpoint = Math.atan(dy / dx);
                  }
                rotationVal = DriveController.calculate(pose.getRotation().getRadians(), (setpoint));
            }
            LED.getInstance().isAligned = Math.abs(DriveController.getPositionError()) < Math.toRadians(3.);
            rotationVal = Math.signum(rotationVal) * Math.max(Math.abs(rotationVal),0.25); //0.02
            if (Math.abs(DriveController.getPositionError()) < Math.toRadians(3.))
                rotationVal = 0.0;

        } else {
            LED.getInstance().isAligned = false;
            saw = false;
        }
        // Drive normally
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal),
                rotationVal,
                !robotCentricSup.getAsBoolean(),
                true,
                maxSpeed);

    }
}
