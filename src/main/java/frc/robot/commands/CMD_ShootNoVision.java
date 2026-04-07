package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
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
  private final BooleanSupplier slowStow;

  private static final double kStowVoltage = -0.85;
  private static final double kSlowStowVoltage = -0.75;

  private final Timer timer = new Timer();

  // Default constructor — always stows at normal speed
  public CMD_ShootNoVision(
      Conveyor conveyor, Hood hood, Intake intake, Kicker kicker, Shootable shooter) {
    this(
        conveyor,
        hood,
        intake,
        kicker,
        shooter,
        () -> Math.toRadians(20000),
        () -> 33,
        () -> true,
        () -> false);
  }

  // Constructor with custom hood/shooter suppliers — always stows at normal speed
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

  // Full constructor — includes operator intake behavior control
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
    this.slowStow = slowStow;

    addRequirements(conveyor, hood, intake, kicker, shooter);
  }

  @Override
  public void initialize() {
    shooting = false;
    timer.reset();
    timer.stop();
  }

  @Override
  public void execute() {
    shooter.setReference(shooterSupplier.getAsDouble());
    hood.setReference(hoodSupplier.getAsDouble());

    if (shooter.isReady() && hood.atReference() && !shooting) {
      conveyor.setVoltage(ConveyorConstants.kConvey);
      kicker.setVoltage(KickerConstants.kKick);
      shooting = true;
      shooter.startShooting();
      timer.start();
    }

    if (shooting && timer.hasElapsed(2)) {
      intake.setExtenderReference(ExtenderConstants.kHome);
    }

    // if (shooting
    //     && stowIntakeOnShoot.getAsBoolean()
    //     && intake.getExtenderPosition() > ExtenderConstants.kStow) {
    //   // Lift the intake out of the shot path — slowly if the hopper is full to avoid squishing
    // //   intake.setExtenderVoltage(slowStow.getAsBoolean() ? kSlowStowVoltage : kStowVoltage);
    // //   intake.setVoltage(2);
    // } else if (!stowIntakeOnShoot.getAsBoolean()) {
    //   if (!intake.getExtenderInPosition()) {
    //     // Drive extender down to extended position
    //     intake.setExtenderReference(ExtenderConstants.kExtended);
    //   } else {
    //     // Once down, cut extender voltage so it can fold back if bumped,
    //     // and run the roller so we're ready to intake immediately after the shot
    //     intake.setExtenderVoltage(0);
    //     intake.setVoltage(IntakeConstants.kOn);
    //   }
    // } else {
    //   intake.setExtenderVoltage(0);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setReference(0);
    hood.setReference(HoodConstants.kMinPos);
    conveyor.setVoltage(ConveyorConstants.kOff);
    kicker.setVoltage(KickerConstants.kOff);
    intake.setExtenderReference(intake.getExtenderPosition());
    intake.setVoltage(0);
    shooter.stopShooting();
  }
}
