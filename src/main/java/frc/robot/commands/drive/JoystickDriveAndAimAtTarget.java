package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.CustomPIDs.ChassisHeadingController;
import frc.robot.utils.CustomPIDs.MapleJoystickDriveInput;
import frc.robot.utils.CustomPIDs.MapleShooterOptimization;
import java.util.function.Supplier;

public class JoystickDriveAndAimAtTarget {

  // Tune this if ±3 degrees is too much or too little
  private static final double MOVING_ANGLE_OFFSET_DEGREES = 3.0;

  public static Command driveAndAimAtTarget(
      MapleJoystickDriveInput input,
      HolonomicDriveSubsystem driveSubsystem,
      Supplier<Translation2d> targetPositionSupplier,
      MapleShooterOptimization shooterOptimization,
      double pilotInputMultiplier,
      boolean finishWhenComplete) {
    return new FunctionalCommand(
        // Init: set heading request with no offset
        () ->
            ChassisHeadingController.getInstance()
                .setHeadingRequest(
                    new ChassisHeadingController.FaceToTargetRequest(
                        targetPositionSupplier, shooterOptimization)),
        // Execute: recompute offset every loop based on current movement direction
        () -> {
          double rawX = input.joystickXSupplier.getAsDouble();
          double rawY = input.joystickYSupplier.getAsDouble();
          boolean isMoving = Math.abs(rawX) > 0.1 || Math.abs(rawY) > 0.1;

          double offsetDegrees =
              isMoving
                  ? calculateAngleOffset(
                      rawX,
                      rawY,
                      driveSubsystem.getPose().getTranslation(),
                      targetPositionSupplier.get())
                  : 0.0;

          ChassisHeadingController.getInstance()
              .setHeadingRequest(
                  new ChassisHeadingController.FaceToTargetRequest(
                      targetPositionSupplier, shooterOptimization, offsetDegrees));

          if (isMoving) {
            execute(driveSubsystem, input, pilotInputMultiplier);
          } else {
            driveSubsystem.activeXLock();
          }
        },
        (interrupted) ->
            ChassisHeadingController.getInstance()
                .setHeadingRequest(new ChassisHeadingController.NullRequest()),
        () -> finishWhenComplete && ChassisHeadingController.getInstance().atSetPoint(),
        driveSubsystem);
  }

  public static Command driveAndAimAtDirection(
      MapleJoystickDriveInput input,
      HolonomicDriveSubsystem driveSubsystem,
      Supplier<Rotation2d> rotationTarget,
      double pilotInputMultiplier,
      boolean finishWhenComplete) {
    return new FunctionalCommand(
        () ->
            ChassisHeadingController.getInstance()
                .setHeadingRequest(
                    new ChassisHeadingController.FaceToRotationRequest(rotationTarget.get())),
        () -> {
          double rawX = input.joystickXSupplier.getAsDouble();
          double rawY = input.joystickYSupplier.getAsDouble();
          boolean isMoving = Math.abs(rawX) > 0.1 || Math.abs(rawY) > 0.1;

          // For a fixed rotation target, offset based on lateral joystick only
          double offsetDegrees = isMoving ? calculateAngleOffsetFromJoystick(rawX, rawY) : 0.0;

          ChassisHeadingController.getInstance()
              .setHeadingRequest(
                  new ChassisHeadingController.FaceToRotationRequest(
                      rotationTarget.get().plus(Rotation2d.fromDegrees(offsetDegrees))));

          if (isMoving) {
            execute(driveSubsystem, input, pilotInputMultiplier);
          } else {
            driveSubsystem.activeXLock();
          }
        },
        (interrupted) ->
            ChassisHeadingController.getInstance()
                .setHeadingRequest(new ChassisHeadingController.NullRequest()),
        () -> finishWhenComplete && ChassisHeadingController.getInstance().atSetPoint(),
        driveSubsystem);
  }

  /**
   * Calculates a heading offset based on how laterally the robot is moving relative to the target.
   * Moving perpendicular to the target line gives the full ±3°. Moving directly toward/away gives
   * 0°. Everything else scales smoothly between.
   *
   * <p>Positive = moving left of target line → offset rotates aim left Negative = moving right of
   * target line → offset rotates aim right
   */
  private static double calculateAngleOffset(
      double rawX, double rawY, Translation2d robotPosition, Translation2d targetPosition) {

    // Angle from robot toward target (field frame)
    Translation2d toTarget = targetPosition.minus(robotPosition);
    double targetAngleRad = Math.atan2(toTarget.getY(), toTarget.getX());

    // Angle the robot is moving (joystick: Y = forward, X = strafe)
    double moveAngleRad = Math.atan2(rawY, rawX);

    // sin of the difference = lateral component (-1 to +1)
    // 0 when moving toward/away, ±1 when moving fully perpendicular
    double lateralComponent = Math.sin(moveAngleRad - targetAngleRad);

    return lateralComponent * MOVING_ANGLE_OFFSET_DEGREES;
  }

  /**
   * Simpler offset for driveAndAimAtDirection where there is no field target position. Uses raw
   * joystick X (strafe) as a proxy for lateral movement.
   */
  private static double calculateAngleOffsetFromJoystick(double rawX, double rawY) {
    double magnitude = Math.hypot(rawX, rawY);
    if (magnitude < 0.1) return 0.0;
    double normalizedLateral = rawX / magnitude;
    return -normalizedLateral * MOVING_ANGLE_OFFSET_DEGREES;
  }

  public static void execute(
      HolonomicDriveSubsystem driveSubsystem,
      MapleJoystickDriveInput input,
      double pilotInputMultiplier) {
    driveSubsystem.runDriverStationCentricChassisSpeeds(
        input.getJoystickChassisSpeeds(
            driveSubsystem.getChassisMaxLinearVelocityMetersPerSec() * pilotInputMultiplier, 0),
        true);
  }
}
