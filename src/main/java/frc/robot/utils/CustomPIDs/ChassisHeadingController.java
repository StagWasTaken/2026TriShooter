package frc.robot.utils.CustomPIDs;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.utils.CustomPIDs.DriveControlLoops.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import org.ironmaple.utils.mathutils.MapleCommonMath;
import org.littletonrobotics.junction.Logger;

public class ChassisHeadingController {

  public abstract static class ChassisHeadingRequest {}

  public static class FaceToRotationRequest extends ChassisHeadingRequest {
    public final Rotation2d rotationTarget;

    public FaceToRotationRequest(Rotation2d rotationTarget) {
      this.rotationTarget = rotationTarget;
    }
  }

  public static class FaceToTargetRequest extends ChassisHeadingRequest {
    public final Supplier<Translation2d> target;
    public final MapleShooterOptimization shooterOptimization;
    public final double angleOffsetDegrees;

    // Original constructor — unchanged, defaults offset to 0
    public FaceToTargetRequest(
        Supplier<Translation2d> target, MapleShooterOptimization shooterOptimization) {
      this(target, shooterOptimization, 0.0);
    }

    // New constructor with offset
    public FaceToTargetRequest(
        Supplier<Translation2d> target,
        MapleShooterOptimization shooterOptimization,
        double angleOffsetDegrees) {
      this.target = target;
      this.shooterOptimization = shooterOptimization;
      this.angleOffsetDegrees = angleOffsetDegrees;
    }
  }

  public static class NullRequest extends ChassisHeadingRequest {}

  private final TrapezoidProfile chassisRotationProfile;
  private final MaplePIDController chassisRotationCloseLoop;
  private final double maxAngularVelocityRadPerSec;
  private ChassisHeadingRequest headingRequest;
  private TrapezoidProfile.State chassisRotationState;

  public ChassisHeadingController(
      TrapezoidProfile.Constraints chassisRotationConstraints,
      MaplePIDController.MaplePIDConfig chassisRotationCloseLoopConfig,
      Rotation2d chassisInitialFacing) {
    this.chassisRotationProfile = new TrapezoidProfile(chassisRotationConstraints);
    this.chassisRotationCloseLoop = new MaplePIDController(chassisRotationCloseLoopConfig);
    this.headingRequest = new NullRequest();
    this.maxAngularVelocityRadPerSec = chassisRotationConstraints.maxVelocity;
    this.chassisRotationState = new TrapezoidProfile.State(chassisInitialFacing.getRadians(), 0);
  }

  public void setHeadingRequest(ChassisHeadingRequest newRequest) {
    this.headingRequest = newRequest;
  }

  public OptionalDouble calculate(ChassisSpeeds measuredSpeedsFieldRelative, Pose2d robotPose) {
    if (headingRequest instanceof FaceToRotationRequest faceToRotationRequest) {
      return OptionalDouble.of(
          calculateFaceToRotation(robotPose, faceToRotationRequest.rotationTarget, 0));
    }

    if (headingRequest instanceof FaceToTargetRequest faceToTargetRequest)
      return OptionalDouble.of(
          calculateFaceToTarget(
              measuredSpeedsFieldRelative,
              robotPose,
              faceToTargetRequest.target.get(),
              faceToTargetRequest.shooterOptimization,
              faceToTargetRequest.angleOffsetDegrees)); // <-- passes offset through

    chassisRotationState =
        new TrapezoidProfile.State(
            robotPose.getRotation().getRadians(),
            measuredSpeedsFieldRelative.omegaRadiansPerSecond);

    log(robotPose, robotPose.getRotation());
    atSetPoint = false;
    return OptionalDouble.empty();
  }

  // Added angleOffsetDegrees parameter
  private double calculateFaceToTarget(
      ChassisSpeeds measuredSpeedsFieldRelative,
      Pose2d robotPose,
      Translation2d targetPosition,
      MapleShooterOptimization shooterOptimization,
      double angleOffsetDegrees) {

    final Translation2d targetMovingSpeed =
        new Translation2d(
            -measuredSpeedsFieldRelative.vxMetersPerSecond,
            -measuredSpeedsFieldRelative.vyMetersPerSecond);

    // Apply the offset after computing the base targeted rotation
    final Rotation2d targetedRotation =
        (shooterOptimization == null
                ? targetPosition.minus(robotPose.getTranslation()).getAngle()
                : shooterOptimization.getShooterFacing(
                    targetPosition, robotPose.getTranslation(), measuredSpeedsFieldRelative))
            .plus(Rotation2d.fromDegrees(angleOffsetDegrees)); // <-- offset applied here

    final Rotation2d targetMovingDirection = targetMovingSpeed.getAngle();
    final Rotation2d positiveRotationTangentDirection =
        targetPosition
            .minus(robotPose.getTranslation())
            .getAngle()
            .rotateBy(Rotation2d.fromDegrees(90));

    final double tangentVelocity =
        targetMovingDirection.minus(positiveRotationTangentDirection).getCos()
            * targetMovingSpeed.getNorm();

    final double distanceToTarget = targetPosition.minus(robotPose.getTranslation()).getNorm();

    final double feedforwardAngularVelocity = tangentVelocity / distanceToTarget;

    return calculateFaceToRotation(robotPose, targetedRotation, feedforwardAngularVelocity);
  }

  private double calculateFaceToRotation(
      Pose2d robotPose, Rotation2d targetedRotation, double desiredAngularVelocityRadPerSec) {
    TrapezoidProfile.State goalState = getGoalState(targetedRotation);
    chassisRotationState =
        chassisRotationProfile.calculate(Robot.defaultPeriodSecs, chassisRotationState, goalState);

    final double unwrappedCurrentRotation =
        goalState.position - targetedRotation.minus(robotPose.getRotation()).getRadians();

    final double feedBackSpeed =
        chassisRotationCloseLoop.calculate(unwrappedCurrentRotation, chassisRotationState.position);

    final double feedForwardSpeedRadPerSec =
        Math.abs(targetedRotation.minus(robotPose.getRotation()).getDegrees()) < 15
            ? desiredAngularVelocityRadPerSec
            : chassisRotationState.velocity;

    log(robotPose, targetedRotation);

    return MapleCommonMath.constrainMagnitude(
        feedBackSpeed + feedForwardSpeedRadPerSec, maxAngularVelocityRadPerSec);
  }

  private TrapezoidProfile.State getGoalState(Rotation2d targetedRotation) {
    final Rotation2d difference =
        targetedRotation.minus(Rotation2d.fromRadians(chassisRotationState.position));
    final double goal = chassisRotationState.position + difference.getRadians();
    return new TrapezoidProfile.State(goal, 0);
  }

  private boolean atSetPoint = false;

  private void log(Pose2d robotPose, Rotation2d requestedRotation) {
    Logger.recordOutput(
        "ChassisHeadingController/Requested",
        new Pose2d(robotPose.getTranslation(), requestedRotation));
    Logger.recordOutput(
        "ChassisHeadingController/CurrentState",
        new Pose2d(
            robotPose.getTranslation(), Rotation2d.fromRadians(chassisRotationState.position)));
    final Rotation2d error = requestedRotation.minus(robotPose.getRotation());
    Logger.recordOutput("ChassisHeadingController/Error", error.getDegrees());
    atSetPoint = Math.abs(error.getRadians()) < Math.toRadians(3);
  }

  public boolean atSetPoint() {
    return atSetPoint;
  }

  private static ChassisHeadingController instance = null;

  public static ChassisHeadingController getInstance() {
    if (instance == null)
      instance =
          new ChassisHeadingController(
              new TrapezoidProfile.Constraints(
                  ANGULAR_VELOCITY_SOFT_CONSTRAIN.in(RadiansPerSecond),
                  ANGULAR_ACCELERATION_SOFT_CONSTRAIN.in(RadiansPerSecondPerSecond)),
              DriveControlLoops.CHASSIS_ROTATION_CLOSE_LOOP,
              new Rotation2d());

    return instance;
  }

  public double getAbsoluteHeadingErrorDegrees() {
    return Math.abs(chassisRotationCloseLoop.getError());
  }

  public void resetToCurrentPose(Pose2d currentPose) {
    chassisRotationState = new TrapezoidProfile.State(currentPose.getRotation().getRadians(), 0);
  }
}
