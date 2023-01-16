package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {

    // Configuration for swerve drive motors
    public static final TalonFXConfiguration swerveDriveFXConfig;
    static {
        swerveDriveFXConfig = new TalonFXConfiguration();

        swerveDriveFXConfig.voltageCompSaturation = 12.0;
        swerveDriveFXConfig.neutralDeadband = 0.0;

        swerveDriveFXConfig.slot0.kP = 0.10;     // Team364 uses 0.10             // CALIBRATED
        swerveDriveFXConfig.slot0.kI = 0.0;
        swerveDriveFXConfig.slot0.kD = 0.005;                                   // CALIBRATED
        swerveDriveFXConfig.slot0.kF = 0.0;     // Use arbitrary FF instead
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = 0.0;     // Team364 uses 0.25        // TODO calibrate
        swerveDriveFXConfig.closedloopRamp = 0.0;                               // TODO calibrate

        // Supply current limit is typically used to prevent breakers from tripping.
        swerveDriveFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);

        // Stator current limit can be used to limit acceleration, torque, braking (when in brake mode), and motor heating.
        // swerveDriveFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        //     enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
    }

    // Configuration for swerve angle motors
    public static final TalonFXConfiguration swerveAngleFXConfig;
    static {
        swerveAngleFXConfig = new TalonFXConfiguration();

        swerveAngleFXConfig.voltageCompSaturation = 12.0;
        swerveAngleFXConfig.neutralDeadband = 0.0;

        swerveAngleFXConfig.slot0.kP = 0.15;     // CALIBRATED = 0.15.  Team364 uses 0.60.
        swerveAngleFXConfig.slot0.kI = 0.0;
        swerveAngleFXConfig.slot0.kD = 3.0;     // CALIBRATED = 3.0.  Team364 uses 12.0.
        swerveAngleFXConfig.slot0.kF = 0.0;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveAngleFXConfig.openloopRamp = 0.1;
        swerveAngleFXConfig.closedloopRamp = 0.1;   // CALIBRATED = 0.1.

        swerveAngleFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true, 25, 40, 0.1);
    }

    // Configuration for swerve angle CanCoders
    public static final CANCoderConfiguration swerveCanCoderConfig;
    static {
        swerveCanCoderConfig = new CANCoderConfiguration();

        // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

}