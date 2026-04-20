package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_HomeIntake extends Command {
  private final Intake intake;
  private final Debouncer stalledDebouncer = new Debouncer(0.1);

  public CMD_HomeIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    stalledDebouncer.calculate(false); // flush debouncer state
    intake.setExtenderVoltage(0);
  }

  @Override
  public void execute() {
    // Drive slowly toward hard stop
    intake.setExtenderVoltage(ExtenderConstants.kHomingVoltage);
  }

  @Override
  public boolean isFinished() {
    // Done when velocity stays near zero — i.e. we've hit the hard stop
    return stalledDebouncer.calculate(
        Math.abs(intake.getExtenderVelocity()) < ExtenderConstants.kHomingVelocityThreshold);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setVoltage(0);
    if (!interrupted) {
      // Zero the encoder at the hard stop position
      intake.resetEncoder();
    }
  }
}
