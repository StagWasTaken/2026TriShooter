package frc.robot.subsystems.drum;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DrumConfig {
  public static final SparkMaxConfig topLeftLeaderConfig = new SparkMaxConfig();
  public static final SparkMaxConfig bottomLeftFollowerConfig = new SparkMaxConfig();
  public static final SparkMaxConfig topRightFollowerConfig = new SparkMaxConfig();
  public static final SparkMaxConfig bottomRightFollowerConfig = new SparkMaxConfig();

  static {
    // Leader — full PID/FF config, all other motors follow this one
    topLeftLeaderConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(DrumConstants.kInverted)
        .smartCurrentLimit(50)
        .voltageCompensation(12.0);
    topLeftLeaderConfig
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(8);
    topLeftLeaderConfig
        .closedLoop
        .pid(DrumConstants.kP, 0.0, DrumConstants.kD, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(DrumConstants.kMinOutput, DrumConstants.kMaxOutput)
        .positionWrappingEnabled(false);

    // Followers — just set follow relationship and inversion, PID/FF is handled by leader
    bottomLeftFollowerConfig
        .follow(DrumConstants.kTopLeftLeaderCanId, DrumConstants.kFollowerAInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .voltageCompensation(12.0);
    bottomLeftFollowerConfig
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(8);

    topRightFollowerConfig
        .follow(DrumConstants.kTopLeftLeaderCanId, DrumConstants.kFollowerBInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .voltageCompensation(12.0);
    topRightFollowerConfig
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(8);

    bottomRightFollowerConfig
        .follow(DrumConstants.kTopLeftLeaderCanId, DrumConstants.kFollowerCInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .voltageCompensation(12.0);
    bottomRightFollowerConfig
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(8);
  }
}
