package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_Stow extends Command {
  private final Intake intake;

  public CMD_Stow(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setVoltage(IntakeConstants.kOff);

    intake.setExtenderReference(ExtenderConstants.kStow);
  }

  @Override
  public boolean isFinished() {
    return intake.getExtenderInPosition();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) return;
  }
}
