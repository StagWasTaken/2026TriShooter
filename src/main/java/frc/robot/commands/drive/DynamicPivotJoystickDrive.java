package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.custompids.MapleJoystickDriveInput;

public class DynamicPivotJoystickDrive extends JoystickDrive {
  private final MapleJoystickDriveInput input;
  private final HolonomicDriveSubsystem driveSubsystem;

  public DynamicPivotJoystickDrive(
      MapleJoystickDriveInput input,
      boolean useDriverStationCentric,
      java.util.function.IntSupplier pov,
      HolonomicDriveSubsystem driveSubsystem) {
    super(input, () -> useDriverStationCentric, pov, driveSubsystem);
    this.input = input;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void execute() {
    Rotation2d robotRotation = driveSubsystem.getFacing();

    // 1. Get speeds exactly as they are sent to a normal field-centric drive
    ChassisSpeeds fieldSpeeds =
        input.getJoystickChassisSpeeds(
            driveSubsystem.getChassisMaxLinearVelocityMetersPerSec(),
            driveSubsystem.getChassisMaxAngularVelocity() * 0.7);

    // 2. Identify "Forward" and "Right" from the DRIVER'S perspective
    boolean driverWantsMovingAway = fieldSpeeds.vxMetersPerSecond >= 0;
    boolean driverWantsRotatingRight = fieldSpeeds.omegaRadiansPerSecond > 0;

    // 3. Find the Robot-Relative Pivot Point
    // We look for the corner that is physically "Back-Left" relative to the FIELD
    // and map it back to the robot's internal coordinate system.
    Translation2d pivotPoint = new Translation2d(); // Default to center

    if (Math.abs(fieldSpeeds.omegaRadiansPerSecond) > 0.1) {
      // We want to pivot around a corner that is "Away" from the direction of rotation.
      // If rotating Right, we want to anchor a point on the LEFT side of the field.

      double targetFieldX = driverWantsMovingAway ? -1.0 : 1.0;
      double targetFieldY = driverWantsRotatingRight ? 1.0 : -1.0;

      // This Translation represents the corner of the robot we want to anchor,
      // relative to the center of the field-aligned robot.
      Translation2d fieldRelativeCorner = new Translation2d(targetFieldX, targetFieldY);

      // ROTATE this field-corner back into the ROBOT'S coordinate system
      // This is the "Magic Step" that fixes the 180-degree inversion.
      pivotPoint = fieldRelativeCorner.rotateBy(robotRotation.unaryMinus());

      // Scale the pivot point to the actual size of your robot (half-track-width)
      // Assuming your modules are roughly 0.3m to 0.5m from center
      double robotRadius = DriveConstants.moduleTranslations[0].getNorm();
      pivotPoint =
          new Translation2d(
              Math.copySign(robotRadius, pivotPoint.getX()),
              Math.copySign(robotRadius, pivotPoint.getY()));
    }

    // 4. Convert field speeds to robot speeds (Standard for CoR usage)
    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vxMetersPerSecond,
            fieldSpeeds.vyMetersPerSecond,
            fieldSpeeds.omegaRadiansPerSecond,
            robotRotation);

    // 5. Drive
    driveSubsystem.runRobotCentricChassisSpeeds(robotSpeeds, pivotPoint);
  }
}
