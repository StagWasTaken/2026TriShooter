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
  private final MapleJoystickDriveInput driveSupplier;
  private final Supplier<Translation2d> targetSupplier;

  private final BooleanSupplier stowIntakeOnShoot, slowStow;

  private boolean shooting;
  private final Debouncer atSetpointDebouncer = new Debouncer(0.04);
  private final Timer timer = new Timer();
  private Command driveCommand;

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

  public CMD_Shoot(
      Drive drive,
      Supplier<Translation2d> targetSupplier,
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shootable shooter) {
    this(
        drive,
        null,
        targetSupplier,
        conveyor,
        hood,
        intake,
        kicker,
        shooter,
        () -> true,
        () -> false);
  }

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

  private ShootingParams getShootingParamsWithPrediction() {
    double distMeters = targetSupplier.get().getDistance(drive.getPose().getTranslation());
    ShootingParams initialParams =
        Robot.CURRENT_ROBOT == RobotName.HYDRA
            ? ShooterConstants.getShootingParams(distMeters)
            : DrumConstants.getShootingParams(distMeters);

    Translation2d predictedPos = getPredictedPosition(initialParams.tofSeconds());
    double predictedDist = targetSupplier.get().getDistance(predictedPos);

    return Robot.CURRENT_ROBOT == RobotName.HYDRA
        ? ShooterConstants.getShootingParams(predictedDist)
        : DrumConstants.getShootingParams(predictedDist);
  }

  @Override
  public void initialize() {
    shooting = false;
    atSetpointDebouncer.calculate(false); // flush debouncer state
    timer.reset();
    timer.start();

    ChassisHeadingController.getInstance()
        .setHeadingRequest(new ChassisHeadingController.NullRequest());
    ChassisHeadingController.getInstance().resetToCurrentPose(drive.getPose());

    if (driveSupplier != null) {
      driveCommand =
          JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
              driveSupplier,
              drive,
              targetSupplier::get,
              DrumConstants.kShooterOptimization,
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
    if ((timer.hasElapsed(1) || atSetpointDebouncer.calculate(shooter.isReady()))
        && hood.atReference()
        && driveReady
        && !shooting) {
      conveyor.setVoltage(ConveyorConstants.kConvey);
      kicker.setVoltage(KickerConstants.kKick);
      shooting = true;
      shooter.startShooting();
    }

    if (shooting) {
      if (!stowIntakeOnShoot.getAsBoolean()) {
        // Keep it down logic: use homing voltage to ensure it's on the floor
        intake.setExtenderReference(ExtenderConstants.kExtended);
        intake.setReference(IntakeConstants.kIntake);
      } else {
        // Stow logic: Use reverse homing voltage (UP)
        // Since we removed getExtenderPosition, we run voltage toward the home stop
        double stowVolts =
            slowStow.getAsBoolean()
                ? ExtenderConstants.kSlowStowVolts
                : ExtenderConstants.kStowVolts;
        if (intake.getExtenderPosition() > ExtenderConstants.kStow) {
          intake.setExtenderVoltage(stowVolts); // Assuming negative is UP/STOW
        } else {
          intake.setExtenderVoltage(0.0);
        }
        intake.setVoltage(2); // Keep rollers spinning slightly to clear notes
      }
    } else {
      intake.setExtenderVoltage(0.1); // Small holding voltage to keep it from flopping
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

    // Stop rollers and kill extender voltage
    intake.setVoltage(0);
    intake.setExtenderVoltage(0);
  }
}
