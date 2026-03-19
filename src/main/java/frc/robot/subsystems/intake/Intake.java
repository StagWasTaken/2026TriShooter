package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final SysIdRoutine intakeSysIdRoutine;
  private final SysIdRoutine intakeExtenderSysIdRoutine;

  public Intake(IntakeIO io) {
    this.io = io;
    this.intakeSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("/Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()), null, this));

    this.intakeExtenderSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("/Intake/ExtenderSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()), null, this));
  }

  public SysIdRoutine getIntakeSysIdRoutine() {
    return intakeSysIdRoutine;
  }

  public SysIdRoutine getExtenderSysIdRoutine() {
    return intakeExtenderSysIdRoutine;
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

  public Command runVoltage(double voltage) {
    return Commands.runOnce(() -> setVoltage(voltage), this);
  }

  public void setReference(double velocity) {
    io.setReference(velocity);
  }

  public double getExtenderReference() {
    return io.getExtenderReference();
  }

  public double getExtenderVelocity() {
    return io.getExtenderVelocity();
  }

  public double getExtenderCurrent() {
    return io.getExtenderCurrent();
  }

  public double getExtenderVoltage() {
    return io.getExtenderVoltage();
  }

  public void setExtenderVoltage(double voltage) {
    io.setExtenderVoltage(voltage);
  }

  public void setExtenderReference(double angRad) {
    io.setExtenderReference(angRad);
  }

  public Command setExtenderTargetAngle(double angRad) {
    return Commands.runOnce(() -> setExtenderReference(angRad), this);
  }

  public double getExtenderPosition() {
    return io.getExtenderPosition();
  }

  public boolean getExtenderInPosition() {
    return io.getExtenderInPosition();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs(this.getName(), inputs);
  }
}
