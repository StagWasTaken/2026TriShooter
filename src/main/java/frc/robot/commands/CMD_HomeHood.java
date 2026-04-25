package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;

public class CMD_HomeHood extends Command {
  private final Hood hood;
  private final Debouncer stalledDebouncer = new Debouncer(0.1);

  public CMD_HomeHood(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    stalledDebouncer.calculate(false);
    hood.setVoltage(0);
  }

  @Override
  public void execute() {
    hood.setVoltage(HoodConstants.kDeployVoltage);
  }

  @Override
  public boolean isFinished() {
    return stalledDebouncer.calculate(
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
