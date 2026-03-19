package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_Intake extends Command {
  private final Intake intake;

  public CMD_Intake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.setExtenderReference(ExtenderConstants.kExtended);
  }

  @Override
  public boolean isFinished() {
    return intake.getExtenderInPosition();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      return;
    }

    intake.setVoltage(IntakeConstants.kOn);
    intake.setExtenderVoltage(0.1);
  }
}
