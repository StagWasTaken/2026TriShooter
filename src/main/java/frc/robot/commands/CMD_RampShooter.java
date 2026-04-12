package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shootable;
import java.util.function.DoubleSupplier;

public class CMD_RampShooter extends Command {
  private final Shootable shooter;
  private final DoubleSupplier shooterSupplier;

  private final Timer shooterRampTimer = new Timer();
  private static final double kShooterRampDuration = 2;
  private double rampFraction;

  public CMD_RampShooter(Shootable shooter, DoubleSupplier shooterSupplier) {
    this.shooter = shooter;
    this.shooterSupplier = shooterSupplier;
  }

  @Override
  public void initialize() {
    shooterRampTimer.reset();
    shooterRampTimer.start();
  }

  @Override
  public void execute() {
    rampFraction = Math.min(shooterRampTimer.get() / kShooterRampDuration, 1.0);
    shooter.setReference(shooterSupplier.getAsDouble() * rampFraction);
  }

  @Override
  public boolean isFinished() {
    return rampFraction == 1 && shooter.isReady();
  }
}
