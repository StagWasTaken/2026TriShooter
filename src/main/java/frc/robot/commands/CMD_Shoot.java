package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.JoystickDriveAndAimAtTarget;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.utils.custompids.ChassisHeadingController;
import frc.robot.utils.custompids.MapleJoystickDriveInput;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CMD_Shoot extends Command {
  private final Drive drive;
  private final Conveyor conveyor;
  private final Hood hood;
  private final Intake intake;
  private final Kicker kicker;
  private final Shooter shooter;
  private final MapleJoystickDriveInput driveSupplier; // null when using auto constructor
  private final Supplier<Translation2d> targetSupplier;

  // When true, the intake is actively lifted during and after the shot (default behavior).
  // When false, the intake is driven to the extended position and held there with the
  // roller running — useful for shooting while staying ready to intake (e.g. passing).
  // Evaluated dynamically each loop so the operator can change their mind mid-shot.
  private final BooleanSupplier stowIntakeOnShoot;

  // When true, the intake is lifted more slowly to avoid squishing balls in a large hopper.
  // Evaluated dynamically each loop so the operator can change their mind mid-shot.
  private final BooleanSupplier slowStow;

  private static final double kStowVoltage = -1.33;
  private static final double kSlowStowVoltage = -0.75; // tune as needed

  private boolean shooting;
  private final Debouncer atSetpointDebouncer = new Debouncer(0.1);
  private Command driveCommand;

  // Teleop constructor — includes driver joystick input and shoot-on-the-fly prediction
  public CMD_Shoot(
      Drive drive,
      MapleJoystickDriveInput driveSupplier,
      Supplier<Translation2d> targetSupplier,
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shooter shooter,
      BooleanSupplier stowIntakeOnShoot,
      BooleanSupplier slowStow) {
    this.drive = drive;
    this.driveSupplier = driveSupplier;
    this.targetSupplier = targetSupplier;
    this.conveyor = conveyor;
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;
    this.stowIntakeOnShoot = stowIntakeOnShoot;
    this.slowStow = slowStow;

    addRequirements(drive, conveyor, hood, intake, kicker, shooter);
  }

  // Auto constructor — no driver input, always stows intake at normal speed
  public CMD_Shoot(
      Drive drive,
      Supplier<Translation2d> targetSupplier,
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shooter shooter) {
    this.drive = drive;
    this.driveSupplier = null;
    this.targetSupplier = targetSupplier;
    this.conveyor = conveyor;
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;
    this.stowIntakeOnShoot = () -> true;
    this.slowStow = () -> false;

    addRequirements(drive, conveyor, hood, intake, kicker, shooter);
  }

  // Projects the robot's current position forward by tofSeconds to account for
  // robot motion during the time-of-flight of the note
  private Translation2d getPredictedPosition(double tofSeconds) {
    var chassisSpeeds = drive.getMeasuredChassisSpeedsRobotRelative();
    var robotAngle = drive.getPose().getRotation();

    double vxField =
        chassisSpeeds.vxMetersPerSecond * robotAngle.getCos()
            - chassisSpeeds.vyMetersPerSecond * robotAngle.getSin();
    double vyField =
        chassisSpeeds.vxMetersPerSecond * robotAngle.getSin()
            + chassisSpeeds.vyMetersPerSecond * robotAngle.getCos();

    return drive
        .getPose()
        .getTranslation()
        .plus(new Translation2d(vxField * tofSeconds, vyField * tofSeconds));
  }

  // Does two lookups: one at current position to get tof, then
  // a second at the predicted position to get the final shooting params
  private ShootingParams getShootingParamsWithPrediction() {
    double distMeters = targetSupplier.get().getDistance(drive.getPose().getTranslation());
    ShootingParams initialParams = ShooterConstants.getShootingParams(distMeters);

    Translation2d predictedPos = getPredictedPosition(initialParams.tofSeconds());
    double predictedDist = targetSupplier.get().getDistance(predictedPos);

    return ShooterConstants.getShootingParams(predictedDist);
  }

  @Override
  public void initialize() {
    shooting = false;
    atSetpointDebouncer.calculate(false); // flush debouncer state

    ChassisHeadingController.getInstance()
        .setHeadingRequest(new ChassisHeadingController.NullRequest());
    ChassisHeadingController.getInstance().resetToCurrentPose(drive.getPose());

    if (driveSupplier != null) {
      // Teleop: aim at target while allowing driver to translate
      driveCommand =
          JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
              driveSupplier,
              drive,
              targetSupplier::get,
              ShooterConstants.kShooterOptimization,
              0.5,
              false);
    } else {
      // Auto: lock in place and aim, with a timeout in case we never fully align
      driveCommand =
          JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
                  new MapleJoystickDriveInput(() -> 0.0, () -> 0.0, () -> 0.0),
                  drive,
                  targetSupplier::get,
                  null,
                  0.0,
                  true)
              .withTimeout(1);
    }
    driveCommand.initialize();
  }

  @Override
  public void execute() {
    driveCommand.execute();

    ShootingParams shootingParams = getShootingParamsWithPrediction();

    shooter.setReference(shootingParams.shooterReference());
    hood.setReference(shootingParams.hoodReference());

    boolean driveReady =
        atSetpointDebouncer.calculate(ChassisHeadingController.getInstance().atSetPoint());

    // Fire once all subsystems are ready — latch shooting=true so we don't re-trigger
    if (shooter.isReady() && hood.atReference() && driveReady && !shooting) {
      conveyor.setVoltage(ConveyorConstants.kConvey);
      kicker.setVoltage(KickerConstants.kKick);
      shooting = true;
    }

    if (shooting
        && stowIntakeOnShoot.getAsBoolean()
        && intake.getExtenderPosition() > ExtenderConstants.kStow) {
      // Lift the intake out of the shot path — slowly if the hopper is full to avoid squishing
      intake.setExtenderVoltage(slowStow.getAsBoolean() ? kSlowStowVoltage : kStowVoltage);
      intake.setVoltage(2);
    } else if (!stowIntakeOnShoot.getAsBoolean()) {
      if (!intake.getExtenderInPosition()) {
        // Drive extender down to extended position
        intake.setExtenderReference(ExtenderConstants.kExtended);
      } else {
        // Once down, cut extender voltage so it can fold back if bumped,
        // and run the roller so we're ready to intake immediately after the shot
        intake.setExtenderVoltage(0);
        intake.setVoltage(IntakeConstants.kOn);
      }
    } else {
      intake.setExtenderVoltage(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.end(interrupted);
    }

    shooter.setReference(0);
    hood.setReference(HoodConstants.kMinPos);
    conveyor.setVoltage(ConveyorConstants.kOff);
    kicker.setVoltage(KickerConstants.kOff);
    // Hold extender at current position and stop roller
    intake.setExtenderReference(intake.getExtenderPosition());
    intake.setVoltage(0);
  }
}
