package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;

public class CMD_Extend extends Command {
  private final Intake intake;
  private final Conveyor conveyor;
  private final Kicker kicker;
  private final Debouncer stalledDebouncer = new Debouncer(0.1);

  public CMD_Extend(Conveyor conveyor, Intake intake, Kicker kicker) {
    this.conveyor = conveyor;
    this.intake = intake;
    this.kicker = kicker;
    addRequirements(intake, conveyor, kicker);
  }

  @Override
  public void initialize() {
    stalledDebouncer.calculate(false);
    intake.setVoltage(IntakeConstants.kOff);
    conveyor.setVoltage(ConveyorConstants.kOff);
    kicker.setVoltage(KickerConstants.kOff);
  }

  @Override
  public void execute() {
    intake.setExtenderReference(ExtenderConstants.kExtended);
  }

  @Override
  public boolean isFinished() {
    return intake.getExtenderInPosition();
  }

  @Override
  public void end(boolean interrupted) {
    intake.setExtenderVoltage(0.5);

    if (!interrupted) {
      //   intake.resetEncoder();
    }
  }
}
