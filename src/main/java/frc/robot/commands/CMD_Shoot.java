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
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class CMD_Shoot extends Command {
  private final Conveyor conveyor;
  private final Hood hood;
  private final Intake intake;
  private final Kicker kicker;
  private final Shooter shooter;

  private boolean shooting;
  private final Timer timer = new Timer();
  private final DoubleSupplier hoodSupplier, shooterSupplier;

  public CMD_Shoot(
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shooter shooter,
      double hoodAng,
      double shooterVel) {
    this(conveyor, hood, intake, kicker, shooter, () -> hoodAng, () -> shooterVel);
  }

  public CMD_Shoot(
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shooter shooter,
      DoubleSupplier hoodSupplier,
      DoubleSupplier shooterSupplier) {
    this.conveyor = conveyor;
    this.hood = hood;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;
    this.hoodSupplier = hoodSupplier;
    this.shooterSupplier = shooterSupplier;

    addRequirements(conveyor, hood, intake, kicker, shooter);
  }

  @Override
  public void initialize() {
    shooting = false;
    timer.stop();
    timer.reset();

    shooter.setReference(shooterSupplier.getAsDouble());
    hood.setReference(hoodSupplier.getAsDouble());
  }
  
  @Override
  public void end(boolean interrupted) {
    shooter.setReference(0);
    hood.setReference(HoodConstants.kMinPos);
    conveyor.setVoltage(ConveyorConstants.kOff);
    kicker.setVoltage(KickerConstants.kOff);
  }

  @Override
  public void execute() {
    if (shooter.isReady() && hood.atReference() && !shooting) {
      conveyor.setVoltage(ConveyorConstants.kConvey);
      kicker.setVoltage(KickerConstants.kKick);

      intake.setExtenderLowCurrentMode(false);
      intake.setExtenderReference(ExtenderConstants.kStow);

      timer.start();
    }

    if (timer.get() > 0.25 && !shooting) {
      shooting = true;
      timer.reset();
    }
  }
}
