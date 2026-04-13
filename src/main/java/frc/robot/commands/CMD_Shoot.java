package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.RobotName;
import frc.robot.commands.drive.JoystickDriveAndAimAtTarget;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drum.DrumConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shootable;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShootingParams;
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
  private final Shootable shooter;
  private final MapleJoystickDriveInput driveSupplier; // null when using auto constructor
  private final Supplier<Translation2d> targetSupplier;

  // When true, intake fully collapses after shot (default behavior).
  // When false, the intake is driven to the extended position and held there with the
  // roller running — useful for shooting while staying ready to intake (e.g. passing).
  // Evaluated dynamically each loop so the operator can change their mind mid-shot.
  private final BooleanSupplier stowIntakeOnShoot;

  private boolean shooting;
  private final Debouncer atSetpointDebouncer = new Debouncer(0.02);
  private final Timer shootTimer = new Timer();
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
      Shootable shooter,
      BooleanSupplier stowIntakeOnShoot,
      BooleanSupplier slowStow) { // slowStow kept for API compatibility, no longer used
    this.drive = drive;
    this.driveSupplier = driveSupplier;
    this.targetSupplier = targetSupplier;
    this.conveyor = conveyor;
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;
    this.stowIntakeOnShoot = stowIntakeOnShoot;

    addRequirements(drive, conveyor, hood, intake, kicker, shooter);
  }

  // Auto constructor — no driver input, always stows intake
  public CMD_Shoot(
      Drive drive,
      Supplier<Translation2d> targetSupplier,
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shootable shooter) {
    this.drive = drive;
    this.driveSupplier = null;
    this.targetSupplier = targetSupplier;
    this.conveyor = conveyor;
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;
    this.stowIntakeOnShoot = () -> true;

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
    ShootingParams initialParams =
        Robot.CURRENT_ROBOT == RobotName.COMP_BOT
            ? ShooterConstants.getShootingParams(distMeters)
            : DrumConstants.getShootingParams(distMeters);

    Translation2d predictedPos = getPredictedPosition(initialParams.tofSeconds());
    double predictedDist = targetSupplier.get().getDistance(predictedPos);

    return Robot.CURRENT_ROBOT == RobotName.COMP_BOT
        ? ShooterConstants.getShootingParams(predictedDist)
        : DrumConstants.getShootingParams(predictedDist);
  }

  @Override
  public void initialize() {
    shooting = false;
    atSetpointDebouncer.calculate(false);
    shootTimer.reset();
    shootTimer.stop();

    ChassisHeadingController.getInstance()
        .setHeadingRequest(new ChassisHeadingController.NullRequest());
    ChassisHeadingController.getInstance().resetToCurrentPose(drive.getPose());

    if (driveSupplier != null) {
      driveCommand =
          JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
              driveSupplier,
              drive,
              targetSupplier::get,
              ShooterConstants.kShooterOptimization,
              0.5,
              false);
    } else {
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

    if (shooter.isReady() && hood.atReference() && driveReady && !shooting) {
      conveyor.setVoltage(ConveyorConstants.kConvey);
      kicker.setVoltage(KickerConstants.kKick);
      shooting = true;
      shooter.startShooting();
      shootTimer.start();
    }

    if (shooting) {
      if (!stowIntakeOnShoot.getAsBoolean()) {
        if (!intake.getExtenderInPosition()) {
          intake.setExtenderReference(ExtenderConstants.kExtended);
        } else {
          intake.setExtenderVoltage(0);
          intake.setVoltage(IntakeConstants.kOn);
        }
      } else if (shootTimer.hasElapsed(0.5)) {
        if (intake.getExtenderPosition() > ExtenderConstants.kStow) {
          intake.setExtenderVoltage(-1.33);
          intake.setVoltage(2);
        } else {
          intake.setExtenderVoltage(0);
          intake.setVoltage(0);
        }
      } else {
        intake.setExtenderVoltage(0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.end(interrupted);
    }

    shooter.setReference(0);
    shooter.stopShooting();
    hood.setReference(HoodConstants.kMinPos);
    conveyor.setVoltage(ConveyorConstants.kOff);
    kicker.setVoltage(KickerConstants.kOff);
    // Hold extender at current position and stop roller
    intake.setExtenderReference(intake.getExtenderPosition());
    intake.setVoltage(0);
  }
}
