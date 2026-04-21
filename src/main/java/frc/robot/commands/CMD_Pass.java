package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.subsystems.shooter.ShootingParams;
import frc.robot.utils.constants.FieldConstants;
import frc.robot.utils.custompids.ChassisHeadingController;
import frc.robot.utils.custompids.MapleJoystickDriveInput;
import java.util.function.BooleanSupplier;
import org.ironmaple.utils.FieldMirroringUtils;

public class CMD_Pass extends Command {
  private final Drive drive;
  private final Conveyor conveyor;
  private final Hood hood;
  private final Intake intake;
  private final Kicker kicker;
  private final Shootable shooter;
  private final MapleJoystickDriveInput driveSupplier;
  private final BooleanSupplier stowIntakeOnShoot, slowStow;

  private boolean shooting;
  private final Debouncer atSetpointDebouncer = new Debouncer(0.02);
  private final Timer shootTimer = new Timer();
  private Command driveCommand;

  // Current target — updated each loop to closest passing target
  private Translation2d currentTarget;

  public CMD_Pass(
      Drive drive,
      MapleJoystickDriveInput driveSupplier,
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shootable shooter,
      BooleanSupplier stowIntakeOnShoot,
      BooleanSupplier slowStow) {
    this.drive = drive;
    this.driveSupplier = driveSupplier;
    this.conveyor = conveyor;
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;
    this.stowIntakeOnShoot = stowIntakeOnShoot;
    this.slowStow = slowStow;

    addRequirements(drive, conveyor, hood, intake, kicker, shooter);
  }

  private Translation2d getClosestTarget() {
    Translation2d leftTarget =
        FieldMirroringUtils.toCurrentAllianceTranslation(FieldConstants.LeftPassingTarget);
    Translation2d rightTarget =
        FieldMirroringUtils.toCurrentAllianceTranslation(FieldConstants.RightPassingTarget);
    Translation2d robotPos = drive.getPose().getTranslation();
    return robotPos.getDistance(leftTarget) <= robotPos.getDistance(rightTarget)
        ? leftTarget
        : rightTarget;
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

  private ShootingParams getPassingParamsWithPrediction() {
    double distMeters = currentTarget.getDistance(drive.getPose().getTranslation());
    ShootingParams initialParams = DrumConstants.getPassingParams(distMeters);

    Translation2d predictedPos = getPredictedPosition(initialParams.tofSeconds());
    double predictedDist = currentTarget.getDistance(predictedPos);

    return DrumConstants.getPassingParams(predictedDist);
  }

  @Override
  public void initialize() {
    shooting = false;
    atSetpointDebouncer.calculate(false);
    shootTimer.reset();
    shootTimer.stop();

    currentTarget = getClosestTarget();

    ChassisHeadingController.getInstance()
        .setHeadingRequest(new ChassisHeadingController.NullRequest());
    ChassisHeadingController.getInstance().resetToCurrentPose(drive.getPose());

    driveCommand =
        JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
            driveSupplier,
            drive,
            () -> currentTarget,
            DrumConstants.kShooterOptimization,
            0.5,
            false);
    driveCommand.initialize();
  }

  @Override
  public void execute() {
    // Update target each loop so it can switch if robot crosses midpoint
    currentTarget = getClosestTarget();

    driveCommand.execute();

    ShootingParams passingParams = getPassingParamsWithPrediction();
    shooter.setReference(passingParams.shooterReference());
    hood.setReference(passingParams.hoodReference());

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
        intake.setExtenderVoltage(ExtenderConstants.kDeployVoltage);
        intake.setReference(IntakeConstants.kIntake);
      } else {
        double stowVolts =
            slowStow.getAsBoolean() ? DrumConstants.kSlowStowVolts : DrumConstants.kStowVolts;
        intake.setExtenderVoltage(stowVolts);
        intake.setVoltage(2);
      }
    } else {
      intake.setExtenderVoltage(0.1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) driveCommand.end(interrupted);
    shooter.setReference(0);
    shooter.stopShooting();
    hood.setReference(HoodConstants.kMinPos);
    conveyor.setVoltage(ConveyorConstants.kOff);
    kicker.setVoltage(KickerConstants.kOff);
    intake.setExtenderReference(intake.getExtenderPosition());
    intake.setVoltage(0);
  }
}
