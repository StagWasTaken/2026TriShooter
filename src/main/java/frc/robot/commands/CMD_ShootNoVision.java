package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.RobotName;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shootable;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CMD_ShootNoVision extends Command {
  private final Conveyor conveyor;
  private final Hood hood;
  private final Intake intake;
  private final Kicker kicker;
  private final Shootable shooter;

  private boolean shooting;
  private final DoubleSupplier hoodSupplier, shooterSupplier;
  private final BooleanSupplier stowIntakeOnShoot;
  private final Timer timer = new Timer();
  private final Debouncer atSetpointDebouncer = new Debouncer(0.04);

  public CMD_ShootNoVision(
      Conveyor conveyor, Hood hood, Intake intake, Kicker kicker, Shootable shooter) {
    this(
        conveyor,
        hood,
        intake,
        kicker,
        shooter,
        () -> Robot.CURRENT_ROBOT == RobotName.HYDRA ? Math.toRadians(20000) : 3333,
        () -> Robot.CURRENT_ROBOT == RobotName.HYDRA ? 0.433 : 33,
        () -> true,
        () -> false);
  }

  public CMD_ShootNoVision(
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shootable shooter,
      DoubleSupplier shooterRPM,
      DoubleSupplier hoodAngle) {
    this(conveyor, hood, intake, kicker, shooter, shooterRPM, hoodAngle, () -> true, () -> false);
  }

  public CMD_ShootNoVision(
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shootable shooter,
      DoubleSupplier shooterRPM,
      DoubleSupplier hoodAngle,
      BooleanSupplier stowIntakeOnShoot,
      BooleanSupplier slowStow) {
    this.conveyor = conveyor;
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;
    this.shooterSupplier = shooterRPM;
    this.hoodSupplier = hoodAngle;
    this.stowIntakeOnShoot = stowIntakeOnShoot;

    addRequirements(conveyor, hood, intake, kicker, shooter);
  }

  @Override
  public void initialize() {
    shooting = false;
    atSetpointDebouncer.calculate(false); // flush debouncer state
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    shooter.setReference(shooterSupplier.getAsDouble());
    hood.setReference(hoodSupplier.getAsDouble());
    if ((timer.hasElapsed(2.0) || atSetpointDebouncer.calculate(shooter.isReady()))
        && hood.atReference()
        && !shooting) {
      conveyor.setVoltage(ConveyorConstants.kConvey);
      kicker.setVoltage(KickerConstants.kKick);
      shooting = true;
      shooter.startShooting();
    }

    if (shooting) {
      if (!stowIntakeOnShoot.getAsBoolean()) {
        intake.setExtenderProfileConstraints(
            ExtenderConstants.kMaxVel, ExtenderConstants.kMaxAccel);
        intake.setExtenderReference(ExtenderConstants.kExtended);
        intake.setReference(IntakeConstants.kIntake);
      } else {
        intake.setExtenderProfileConstraints(
            ExtenderConstants.kStowProfileMaxVel, ExtenderConstants.kStowProfileMaxAccel);
        intake.setExtenderReference(ExtenderConstants.kHome);
        intake.setVoltage(2.0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setReference(0);
    hood.setReference(HoodConstants.kMinPos);
    conveyor.setVoltage(ConveyorConstants.kOff);
    kicker.setVoltage(KickerConstants.kOff);
    intake.setExtenderVoltage(0);
    intake.setVoltage(0);
    shooter.stopShooting();
  }
}
