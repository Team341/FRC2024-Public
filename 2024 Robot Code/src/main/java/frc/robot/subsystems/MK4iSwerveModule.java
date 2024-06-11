package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.utilities.CTREModuleState;
import frc.robot.utilities.DaisyMath;
import frc.robot.utilities.SwerveModuleConstants;

public class MK4iSwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    public MK4iSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.CANIVORE_NAME);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.CANIVORE_NAME);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.CANIVORE_NAME);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setTurnMotorInverted(boolean isInverted) {
        mAngleMotor.setInverted(isInverted);
    }

    public void setBrakeMode(boolean breakMode) {
        mAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState.angle = Rotation2d
                .fromDegrees(DaisyMath.boundAngleNeg180to180Degrees(desiredState.angle.getDegrees()));

        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void setDesiredStateOverride(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        setAngleDiamond(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            // SmartDashboard.putNumber(moduleNumber+" drive output", percentOutput);
            mDriveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond * (1 / Constants.Swerve.wheelCircumference);
            // SmartDashboard.putNumber(moduleNumber + " drive error", mDriveMotor.getClosedLoopError().getValueAsDouble());
            mDriveMotor.setControl(
                    new VelocityDutyCycle(velocity, 20., false, feedforward.calculate(velocity), 0, false, false,
                            false));
        }
    }
    
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.05))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

                
        mAngleMotor.setControl(new PositionDutyCycle(angle.getRotations()));

        lastAngle = angle;
    }

    private void setAngleDiamond(SwerveModuleState desiredState) {
        Rotation2d angle = desiredState.angle;

        mAngleMotor.setControl(new PositionDutyCycle(angle.getRotations()));

        lastAngle = angle;
    }

    private Rotation2d getAngle() {

        return Rotation2d.fromDegrees(
                DaisyMath.boundAngleNeg180to180Degrees(mAngleMotor.getPosition().getValueAsDouble() * 360));
    }


    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public Rotation2d getCanCoderWithOffset() {
        return Rotation2d.fromDegrees(
                DaisyMath.boundAngleNeg180to180Degrees(
                        angleEncoder.getAbsolutePosition().getValueAsDouble() * 360 - angleOffset.getDegrees()));
    }

    public void resetToAbsolute() {
        double absolutePosition = DaisyMath
                .boundAngleNeg180to180Degrees(getCanCoder().getDegrees() - angleOffset.getDegrees());
        mAngleMotor.setPosition(absolutePosition / 360.);
    }

    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(Constants.CTRE_CONFIGS.swerveCanCoderConfig);

    }

    private void configAngleMotor() {
        mAngleMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.currentLimitConfig);

        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setPosition(0);
        mDriveMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.currentLimitConfig);
    
    }
    public void configDriveMotorTele() {
     
        mDriveMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.currentLimitConfigTeleop);
    
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(mDriveMotor.get() * Constants.Swerve.maxSpeed,
                getAngle());
    }
  

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                mDriveMotor.getPosition().getValueAsDouble() * Constants.Swerve.wheelCircumference,
                getAngle());
    }

    public double getDutyCycle() {
        return mDriveMotor.getSupplyCurrent().getValueAsDouble();

    }


    public double getRawAngleRot() {
        return mAngleMotor.getPosition().getValueAsDouble();
    }
}