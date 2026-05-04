package frc.robot.subsystems.drum;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DrumConfig {
  public static final SparkMaxConfig topLeftConfig = new SparkMaxConfig();
  public static final SparkMaxConfig bottomLeftConfig = new SparkMaxConfig();
  public static final SparkMaxConfig topRightConfig = new SparkMaxConfig();
  public static final SparkMaxConfig bottomRightConfig = new SparkMaxConfig();

  private static void applyCommonConfig(SparkMaxConfig config, boolean inverted) {
    config
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(inverted)
        .smartCurrentLimit(DrumConstants.kCurrentLimit);
    config
        .encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(20);
  }

  static {
    applyCommonConfig(topLeftConfig, DrumConstants.kInverted);
    applyCommonConfig(bottomLeftConfig, DrumConstants.kBottomLeftInverted);
    applyCommonConfig(topRightConfig, DrumConstants.kTopRightInverted);
    applyCommonConfig(bottomRightConfig, DrumConstants.kBottomRightInverted);
  }
}
