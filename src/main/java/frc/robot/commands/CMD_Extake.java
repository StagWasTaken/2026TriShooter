package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;

public class CMD_Extake extends Command {
  private final Conveyor conveyor;
  private final Intake intake;
  private final Kicker kicker;
  private final Timer timer = new Timer();

  public CMD_Extake(Conveyor conveyor, Intake intake, Kicker kicker) {
    this.conveyor = conveyor;
    this.intake = intake;
    this.kicker = kicker;
    addRequirements(conveyor, intake);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.setExtenderProfileConstraints(ExtenderConstants.kMaxVel, ExtenderConstants.kMaxAccel);
    intake.setReference(IntakeConstants.kOff);
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
    intake.setExtenderVoltage(0.0);

    if (!interrupted) {
      conveyor.setVoltage(ConveyorConstants.kExtake);
      intake.setVoltage(IntakeConstants.kExtake);
      kicker.setVoltage(KickerConstants.kExtake);
    }
  }
}
