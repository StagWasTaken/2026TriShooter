package frc.robot.subsystems.drum;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DrumConfig {
  public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
  public static final SparkMaxConfig followerConfigA = new SparkMaxConfig();
  public static final SparkMaxConfig followerConfigB = new SparkMaxConfig();
  public static final SparkMaxConfig followerConfigC = new SparkMaxConfig();

  static {
    // Leader — full PID/FF config, all other motors follow this one
    leaderConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(DrumConstants.kInverted)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
    leaderConfig
        .encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .uvwAverageDepth(4)
        .uvwMeasurementPeriod(10);
    leaderConfig
        .closedLoop
        .pid(DrumConstants.kP, 0.0, DrumConstants.kD, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(DrumConstants.kMinOutput, DrumConstants.kMaxOutput)
        .positionWrappingEnabled(false);

    // Followers — just set follow relationship and inversion, PID/FF is handled by leader
    followerConfigA
        .follow(DrumConstants.kLeaderCanId, DrumConstants.kFollowerAInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);

    followerConfigB
        .follow(DrumConstants.kLeaderCanId, DrumConstants.kFollowerBInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);

    followerConfigC
        .follow(DrumConstants.kLeaderCanId, DrumConstants.kFollowerCInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
  }
}
