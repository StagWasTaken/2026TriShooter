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
    stalledDebouncer.calculate(false); // flush debouncer state
  }

  @Override
  public void execute() {
    // Drive slowly toward hard stop
    hood.setVoltage(HoodConstants.kHomingVoltage);
  }

  @Override
  public boolean isFinished() {
    // Done when velocity stays near zero — i.e. we've hit the hard stop
    return stalledDebouncer.calculate(
        Math.abs(hood.getVelocity()) < HoodConstants.kHomingVelocityThreshold);
  }

  @Override
  public void end(boolean interrupted) {
    hood.setVoltage(0);
    if (!interrupted) {
      // Zero the encoder at the hard stop position
      hood.resetEncoder();
    }
  }
}
