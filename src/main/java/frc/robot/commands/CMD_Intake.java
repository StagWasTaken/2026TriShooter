package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_Intake extends Command {
  private final Intake intake;
  private final Debouncer stalledDebouncer = new Debouncer(0.1);
  private final Timer timer = new Timer();

  public CMD_Intake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    stalledDebouncer.calculate(false);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    intake.setExtenderReference(ExtenderConstants.kExtended);
  }

  @Override
  public boolean isFinished() {
    return intake.getExtenderInPosition() || timer.hasElapsed(0.75);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setExtenderVoltage(0.5);

    if (!interrupted) {
      intake.setReference(IntakeConstants.kIntake);
      //   intake.resetEncoder();
    }
  }
}
