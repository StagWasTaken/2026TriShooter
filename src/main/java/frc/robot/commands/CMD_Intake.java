package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_Intake extends Command {
  private final Intake intake;
  private final Timer timer = new Timer();

  public CMD_Intake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.setExtenderReference(ExtenderConstants.kExtended);
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return intake.getExtenderInPosition() || timer.hasElapsed(1);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      return;
    }

    intake.setReference(IntakeConstants.kIntake);
    intake.setExtenderVoltage(0.1);
  }
}
