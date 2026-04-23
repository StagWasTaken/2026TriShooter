package frc.robot.subsystems.intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class IntakeConfig {
  public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
  public static final SparkFlexConfig secondaryRollerConfig = new SparkFlexConfig();

  public static final SparkMaxConfig intakeExtenderConfig = new SparkMaxConfig();

  static {
    intakeConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(IntakeConstants.kInverted)
        .smartCurrentLimit(40);
    intakeConfig
        .encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .uvwAverageDepth(4)
        .uvwMeasurementPeriod(20);

    secondaryRollerConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kCoast)
        .inverted(IntakeConstants.kSecondaryInverted)
        .smartCurrentLimit(40);
    secondaryRollerConfig
        .encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .uvwAverageDepth(4)
        .uvwMeasurementPeriod(20);

    intakeExtenderConfig
        .disableFollowerMode()
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(40);
    intakeExtenderConfig
        .absoluteEncoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor((Math.PI * 2) / 60)
        .averageDepth(4)
        .inverted(true);
    intakeExtenderConfig
        .closedLoop
        .pid(
            ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD, ClosedLoopSlot.kSlot0)
        .outputRange(ExtenderConstants.kMinOutput, ExtenderConstants.kMaxOutput)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    intakeExtenderConfig
        .closedLoop
        .maxMotion
        .allowedProfileError(ExtenderConstants.kPositionTolerance, ClosedLoopSlot.kSlot0)
        .cruiseVelocity(ExtenderConstants.kMaxVel, ClosedLoopSlot.kSlot0)
        .maxAcceleration(ExtenderConstants.kMaxAccel, ClosedLoopSlot.kSlot0)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);
  }
}
