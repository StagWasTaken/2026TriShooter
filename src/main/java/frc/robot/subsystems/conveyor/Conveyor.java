package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  private final SysIdRoutine sysIdRoutine;

  public Conveyor(ConveyorIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("/Conveyor/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()), null, this));
  }

  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  public double getReference() {
    return io.getReference();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public double getCurrent() {
    return io.getCurrent();
  }

  public double getVoltage() {
    return io.getVoltage();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setReference(double velocity) {
    io.setReference(velocity);
  }

  public Command runVoltage(double voltage) {
    return Commands.run(() -> setVoltage(voltage), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs(this.getName(), inputs);
  }
}
