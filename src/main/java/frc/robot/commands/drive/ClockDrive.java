package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.custom.pids.ChassisHeadingController;
import frc.robot.utils.custom.pids.MapleJoystickDriveInput;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import org.ironmaple.utils.FieldMirroringUtils;

public class ClockDrive extends Command {
  public static final double ROTATION_AXIS_THRESHOLD = 0.5;
  public static final double LINEAR_DEADBAND = 0.05;
  public static final double ROTATION_DEADBAND = 0.05;

  private final HolonomicDriveSubsystem driveSubsystem;
  private final MapleJoystickDriveInput input;
  private final DoubleSupplier rotationXSupplier, rotationYSupplier;
  private final AtomicReference<Optional<Rotation2d>> rotationalTargetOverride;

  private Rotation2d currentDesiredFacing;

  public ClockDrive(
      HolonomicDriveSubsystem driveSubsystem,
      MapleJoystickDriveInput input,
      DoubleSupplier rotationXSupplier,
      DoubleSupplier rotationYSupplier,
      AtomicReference<Optional<Rotation2d>> rotationalTargetOverride) {

    this.driveSubsystem = driveSubsystem;
    this.input = input;
    this.rotationXSupplier = rotationXSupplier;
    this.rotationYSupplier = rotationYSupplier;
    this.rotationalTargetOverride = rotationalTargetOverride;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    currentDesiredFacing = driveSubsystem.getFacing();
  }

  private Translation2d applyRotationDeadband(double x, double y) {
    Translation2d vec = new Translation2d(x, y);
    if (vec.getNorm() < ROTATION_DEADBAND) {
      return new Translation2d(0.0, 0.0);
    }
    return vec;
  }

  @Override
  public void execute() {

    double maxLinear = driveSubsystem.getChassisMaxLinearVelocityMetersPerSec();
    double maxAngular = driveSubsystem.getChassisMaxAngularVelocity();

    ChassisSpeeds pilotInputSpeed = input.getJoystickChassisSpeeds(maxLinear, maxAngular);

    double linMag =
        Math.hypot(pilotInputSpeed.vxMetersPerSecond, pilotInputSpeed.vyMetersPerSecond);

    if (linMag < LINEAR_DEADBAND * maxLinear) {
      pilotInputSpeed.vxMetersPerSecond = 0.0;
      pilotInputSpeed.vyMetersPerSecond = 0.0;
    }

    Optional<Rotation2d> override = rotationalTargetOverride.get();

    if (override.isPresent()) {
      Rotation2d rawTarget = override.get();
      Rotation2d current = driveSubsystem.getFacing();

      double continuous = current.getRadians() + rawTarget.minus(current).getRadians();

      Rotation2d continuousTarget = new Rotation2d(continuous);

      ChassisHeadingController.getInstance()
          .setHeadingRequest(new ChassisHeadingController.FaceToRotationRequest(continuousTarget));

      ChassisSpeeds robotSpeeds = driveSubsystem.getMeasuredChassisSpeedsRobotRelative();

      ChassisSpeeds corrected = new ChassisSpeeds(0.0, 0.0, robotSpeeds.omegaRadiansPerSecond);

      pilotInputSpeed.omegaRadiansPerSecond =
          ChassisHeadingController.getInstance()
              .calculate(corrected, driveSubsystem.getPose())
              .orElse(0);
    } else {
      Translation2d headingVector =
          applyRotationDeadband(rotationXSupplier.getAsDouble(), rotationYSupplier.getAsDouble());

      if (headingVector.getNorm() > ROTATION_AXIS_THRESHOLD) {
        currentDesiredFacing =
            headingVector
                .getAngle()
                .plus(FieldMirroringUtils.getCurrentAllianceDriverStationFacing());
      }

      ChassisHeadingController.getInstance()
          .setHeadingRequest(
              new ChassisHeadingController.FaceToRotationRequest(currentDesiredFacing));
    }

    driveSubsystem.runDriverStationCentricChassisSpeeds(pilotInputSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    ChassisHeadingController.getInstance()
        .setHeadingRequest(new ChassisHeadingController.NullRequest());
  }
}
