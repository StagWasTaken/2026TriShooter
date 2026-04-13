package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterConfig {
  public static final SparkMaxConfig middleShooterConfig = new SparkMaxConfig();
  public static final SparkMaxConfig leftShooterConfig = new SparkMaxConfig();
  public static final SparkMaxConfig rightShooterConfig = new SparkMaxConfig();

  static {
    middleShooterConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(ShooterConstants.kInverted)
        .smartCurrentLimit(40);
    middleShooterConfig
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(4)
        .uvwMeasurementPeriod(10);
    middleShooterConfig
        .closedLoop
        .pid(
            ShooterConstants.kMiddleShooterP,
            0.0,
            ShooterConstants.kMiddleShooterD,
            ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
        .positionWrappingEnabled(false);

    leftShooterConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(ShooterConstants.kInverted)
        .smartCurrentLimit(40);
    leftShooterConfig
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(4)
        .uvwMeasurementPeriod(10);
    leftShooterConfig
        .closedLoop
        .pid(
            ShooterConstants.kLeftShooterP,
            0.0,
            ShooterConstants.kLeftShooterD,
            ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
        .positionWrappingEnabled(false);

    rightShooterConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(ShooterConstants.kInverted)
        .smartCurrentLimit(40);
    rightShooterConfig
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(4)
        .uvwMeasurementPeriod(10);
    rightShooterConfig
        .closedLoop
        .pid(
            ShooterConstants.kRightShooterP,
            0.0,
            ShooterConstants.kRightShooterD,
            ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
        .positionWrappingEnabled(false);
  }
}
