package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;

public class CMD_Extend extends Command {
  private final Conveyor conveyor;
  private final Intake intake;
  private final Kicker kicker;
  private final Timer timer = new Timer();

  public CMD_Extend(Conveyor conveyor, Intake intake, Kicker kicker) {
    this.conveyor = conveyor;
    this.intake = intake;
    this.kicker = kicker;
    addRequirements(conveyor, intake, kicker);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.setExtenderProfileConstraints(ExtenderConstants.kMaxVel, ExtenderConstants.kMaxAccel);
    intake.setReference(IntakeConstants.kOff);
    kicker.setVoltage(KickerConstants.kOff);
  }

  @Override
  public void execute() {
    intake.setExtenderReference(ExtenderConstants.kExtended);
    conveyor.setVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return intake.getExtenderInPosition() || timer.hasElapsed(0.75);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setExtenderVoltage(0.0);
  }
}
