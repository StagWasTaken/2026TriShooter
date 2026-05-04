package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shootable;

public class CMD_HomeHood extends Command {
  private final Hood hood;
  private final Shootable shooter;
  private final Debouncer stalledDebouncer = new Debouncer(0.1);

  public CMD_HomeHood(Hood hood, Shootable shooter) {
    this.hood = hood;
    this.shooter = shooter;
    addRequirements(hood, shooter);
  }

  @Override
  public void initialize() {
    stalledDebouncer.calculate(false);
    hood.setVoltage(0);
    shooter.setVoltage(0.0);
  }

  @Override
  public void execute() {
    if (shooter.getVelocity() < 200) {
      hood.setVoltage(HoodConstants.kDeployVoltage);
    }
  }

  @Override
  public boolean isFinished() {
    return (shooter.getVelocity() < 200)
        && stalledDebouncer.calculate(
            Math.abs(hood.getVelocity()) < HoodConstants.kHomingVelocityThreshold);
  }

  @Override
  public void end(boolean interrupted) {
    hood.setVoltage(0);

    if (!interrupted) {
      hood.resetEncoder();
    }
  }
}
