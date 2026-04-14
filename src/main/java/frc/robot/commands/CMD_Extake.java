package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_Extake extends Command {
  private final Conveyor conveyor;
  private final Intake intake;

  public CMD_Extake(Conveyor conveyor, Intake intake) {
    this.conveyor = conveyor;
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

    intake.setReference(IntakeConstants.kExtake);
    conveyor.setVoltage(ConveyorConstants.kExtake);
    intake.setExtenderVoltage(0.1);
  }
}
