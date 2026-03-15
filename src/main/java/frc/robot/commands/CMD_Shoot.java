package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.RobotName;
import frc.robot.commands.drive.JoystickDriveAndAimAtTarget;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.utils.CustomPIDs.ChassisHeadingController;
import frc.robot.utils.CustomPIDs.MapleJoystickDriveInput;
import frc.robot.utils.constants.FieldConstants;

public class CMD_Shoot extends Command {
  private final Drive drive;
  private final Conveyor conveyor;
  private final Hood hood;
  private final Intake intake;
  private final Kicker kicker;
  private final Shooter shooter;
  private final MapleJoystickDriveInput driveSupplier;

  private boolean shooting;
  private final Timer timer = new Timer();
  private final Debouncer atSetpointDebouncer = new Debouncer(0.5);
  private Command driveCommand;

  public CMD_Shoot(
      Drive drive,
      MapleJoystickDriveInput driveSupplier,
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shooter shooter) {
    this.drive = drive;
    this.driveSupplier = driveSupplier;
    this.conveyor = conveyor;
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;

    addRequirements(drive, conveyor, hood, intake, kicker, shooter);
  }

  @Override
  public void initialize() {
    shooting = false;
    timer.stop();
    timer.reset();

    driveCommand =
        JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
            driveSupplier,
            drive,
            FieldConstants::getHubPose,
            ShooterConstants.kShooterOptimization,
            0.5,
            false);
    driveCommand.initialize();

    // double distMeters =
    // FieldConstants.getHubPose().getDistance(drive.getPose().getTranslation());
    // ShootingParams shootingParams = ShooterConstants.getShootingParams(distMeters);
    // shooter.setReference(shootingParams.shooterReference());
    // hood.setReference(shootingParams.hoodReference());
  }

  @Override
  public void execute() {
    driveCommand.execute();
    shooter.setReference(Math.toRadians(20000));
    hood.setReference(Robot.CURRENT_ROBOT == RobotName.COMP_BOT ? 0.4 : 0.25);

    boolean driveReady =
        atSetpointDebouncer.calculate(ChassisHeadingController.getInstance().atSetPoint());

    if (shooter.isReady() && hood.atReference() && driveReady && !shooting) {
      conveyor.setVoltage(ConveyorConstants.kConvey);
      kicker.setVoltage(KickerConstants.kKick);
      timer.restart();
      shooting = true;
    }

    if (shooting && timer.get() > 1) {
      intake.setExtenderReference(ExtenderConstants.kStow);
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
  }
}
