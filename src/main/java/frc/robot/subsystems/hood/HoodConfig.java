package frc.robot.subsystems.hood;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HoodConfig {
  public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

  static {
    hoodConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kBrake)
        .inverted(HoodConstants.kInverted)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
    hoodConfig
        .absoluteEncoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .averageDepth(4)
        .inverted(true);
    hoodConfig
        .closedLoop
        .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD, ClosedLoopSlot.kSlot0)
        .outputRange(HoodConstants.kMinOutput, HoodConstants.kMaxOutput)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    hoodConfig.softLimit
        .reverseSoftLimit(HoodConstants.kMinPos)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(HoodConstants.kMaxPos)
        .forwardSoftLimitEnabled(true);
    hoodConfig
        .closedLoop
        .maxMotion
        .allowedProfileError(HoodConstants.kTolerance, ClosedLoopSlot.kSlot0)
        .cruiseVelocity(HoodConstants.kMaxVel, ClosedLoopSlot.kSlot0)
        .maxAcceleration(HoodConstants.kMaxAccel)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);
  }
}
