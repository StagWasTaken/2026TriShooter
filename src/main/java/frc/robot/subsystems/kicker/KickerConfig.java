package frc.robot.subsystems.kicker;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerConfig {
  public static final SparkMaxConfig kickerLeaderConfig = new SparkMaxConfig();
  public static final SparkMaxConfig kickerFollowerConfig = new SparkMaxConfig();

  static {
    kickerLeaderConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kBrake)
        .inverted(KickerConstants.kInverted)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
    kickerLeaderConfig
        .encoder
        .positionConversionFactor((Math.PI * 2) / KickerConstants.kGearRatio)
        .velocityConversionFactor((Math.PI * 2) / (60 * KickerConstants.kGearRatio))
        .uvwAverageDepth(4)
        .uvwMeasurementPeriod(16);
    kickerLeaderConfig
        .closedLoop
        .pid(KickerConstants.kP, KickerConstants.kI, KickerConstants.kD)
        .outputRange(KickerConstants.kMinOutput, KickerConstants.kMaxOutput)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    kickerFollowerConfig
        .follow(KickerConstants.kKickerLeadCanId, true)
        .idleMode(IdleMode.kBrake)
        .inverted(KickerConstants.kInverted)
        .voltageCompensation(12.0)
        .smartCurrentLimit(40);
  }
}
