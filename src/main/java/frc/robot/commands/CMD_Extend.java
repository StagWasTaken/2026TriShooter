package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_Extend extends Command {
  private final Intake intake;
  private final Conveyor conveyor;
  private final Debouncer stalledDebouncer = new Debouncer(0.1);

  public CMD_Extend(Conveyor conveyor, Intake intake) {
    this.conveyor = conveyor;
    this.intake = intake;
    addRequirements(intake, conveyor);
  }

  @Override
  public void initialize() {
    stalledDebouncer.calculate(false);
    intake.setVoltage(IntakeConstants.kOff);
    conveyor.setVoltage(ConveyorConstants.kOff);
  }

  @Override
  public void execute() {
    intake.setExtenderVoltage(ExtenderConstants.kDeployVoltage);
  }

  @Override
  public boolean isFinished() {
    return stalledDebouncer.calculate(
        Math.abs(intake.getExtenderVelocity()) < ExtenderConstants.kHomingVelocityThreshold);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setExtenderVoltage(0.5);

    if (!interrupted) {
      intake.resetEncoder();
    }
  }
}
