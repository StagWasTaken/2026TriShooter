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

  private static void applyCommonConfig(SparkMaxConfig config, boolean inverted) {
    config
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(inverted)
        .smartCurrentLimit(50)
        .secondaryCurrentLimit(60)
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(20);
    config
        .closedLoop
        .pid(DrumConstants.kP, 0.0, DrumConstants.kD, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(DrumConstants.kMinOutput, DrumConstants.kMaxOutput)
        .positionWrappingEnabled(false);
  }

  static {
    applyCommonConfig(topLeftLeaderConfig, DrumConstants.kInverted);
    applyCommonConfig(bottomLeftFollowerConfig, DrumConstants.kFollowerAInverted);
    applyCommonConfig(topRightFollowerConfig, DrumConstants.kFollowerBInverted);
    applyCommonConfig(bottomRightFollowerConfig, DrumConstants.kFollowerCInverted);
  }
}
