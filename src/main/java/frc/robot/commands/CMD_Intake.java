package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_Intake extends Command {
  private final Conveyor conveyor;
  private final Intake intake;
  private final Timer timer = new Timer();

  public CMD_Intake(Conveyor conveyor, Intake intake) {
    this.conveyor = conveyor;
    this.intake = intake;
    addRequirements(conveyor, intake);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.setExtenderProfileConstraints(ExtenderConstants.kMaxVel, ExtenderConstants.kMaxAccel);
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

    if (!interrupted) {
      intake.setReference(IntakeConstants.kIntake);
    }
  }
}
