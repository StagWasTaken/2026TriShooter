package frc.robot.subsystems.kicker;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerConfig {
  public static final SparkMaxConfig kickerLeaderConfig = new SparkMaxConfig();
  public static final SparkMaxConfig kickerBottomRightonfig = new SparkMaxConfig();

  static {
    kickerLeaderConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kBrake)
        .inverted(KickerConstants.kInverted)
        .smartCurrentLimit(20);
    kickerLeaderConfig
        .encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .uvwAverageDepth(4)
        .uvwMeasurementPeriod(16);
    kickerLeaderConfig
        .closedLoop
        .pid(KickerConstants.kP, KickerConstants.kI, KickerConstants.kD)
        .outputRange(KickerConstants.kMinOutput, KickerConstants.kMaxOutput)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    kickerBottomRightonfig
        .follow(KickerConstants.kKickerLeadCanId, true)
        .idleMode(IdleMode.kBrake)
        .inverted(KickerConstants.kInverted)
        .smartCurrentLimit(20);
  }
}
