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

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Constants.Swerve.angleEnableCurrentLimit)
                .withSupplyCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
                .withSupplyCurrentThreshold(Constants.Swerve.anglePeakCurrentLimit)
                .withSupplyTimeThreshold(Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.Slot0.kV = Constants.Swerve.angleKV;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Constants.Swerve.driveEnableCurrentLimit)
                .withSupplyCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
                .withSupplyCurrentThreshold(Constants.Swerve.drivePeakCurrentLimit)
                .withSupplyTimeThreshold(Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.driveKV;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;
        swerveDriveFXConfig.OpenLoopRamps = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps = Constants.Swerve.closedLoopRamp;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.canCoderInvert
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
                
        // swerveCanCoderConfig.MagnetSensor.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.MagnetSensor.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}