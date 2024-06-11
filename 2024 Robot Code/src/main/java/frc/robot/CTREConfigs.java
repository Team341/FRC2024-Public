package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;
    public CurrentLimitsConfigs currentLimitConfig;
        public CurrentLimitsConfigs currentLimitConfigTeleop;
    public CurrentLimitsConfigs shooterCurrentLimitConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();
        currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfigTeleop = new CurrentLimitsConfigs();
        shooterCurrentLimitConfig = new CurrentLimitsConfigs();

     
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.anglePIDs.kP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.anglePIDs.kI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.anglePIDs.kD;
        swerveAngleFXConfig.Slot0.kV = Constants.Swerve.anglePIDs.kF;
        swerveAngleFXConfig.Slot0.kS = .0; 
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        swerveAngleFXConfig.MotionMagic.MotionMagicAcceleration = 80.;
        swerveAngleFXConfig.MotionMagic.MotionMagicCruiseVelocity = 160.;

        swerveAngleFXConfig.MotionMagic.MotionMagicJerk = 1600.;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = 21.428571428571427;

        

        // /* Swerve Drive Motor Configuration */
       
        currentLimitConfig.SupplyCurrentLimit = 105;
        currentLimitConfig.StatorCurrentLimit = 60; 
        currentLimitConfig.StatorCurrentLimitEnable  =true;
        currentLimitConfig.SupplyCurrentLimitEnable = true;
        
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        shooterCurrentLimitConfig.SupplyCurrentLimit = 60;


        shooterCurrentLimitConfig.SupplyCurrentLimitEnable = true;
        shooterCurrentLimitConfig.StatorCurrentLimit = 60;
        shooterCurrentLimitConfig.StatorCurrentLimitEnable = true;
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.drivePIDs.kP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.drivePIDs.kI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.drivePIDs.kD;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.drivePIDs.kF;
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = 6.75;
        swerveDriveFXConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = 7.5; // took ~0.20 to go 0 to full voltage (40A)in auto * Might be Duty cycle (percentage based, rather than this nonsense)
        
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.canCoderInvert
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        currentLimitConfigTeleop.StatorCurrentLimit = 80;
                currentLimitConfigTeleop.SupplyCurrentLimit = 105;

        currentLimitConfigTeleop.StatorCurrentLimitEnable  =true;
        currentLimitConfigTeleop.SupplyCurrentLimitEnable = true;
        // swerveCanCoderConfig.initializationStrategy =
        // SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
