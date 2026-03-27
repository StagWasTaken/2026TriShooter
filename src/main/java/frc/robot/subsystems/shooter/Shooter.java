package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  public Shooter(ShooterIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("/Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()), null, this));
  }

  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  public double getReference() {
    return io.getReference();
  }

  public double getMiddleVelocity() {
    return io.getMiddleVelocity();
  }

  public double getMiddleCurrent() {
    return io.getMiddleCurrent();
  }

  public double getMiddleVoltage() {
    return io.getMiddleVoltage();
  }

  public double getLeftVelocity() {
    return io.getLeftVelocity();
  }

  public double getLeftCurrent() {
    return io.getMiddleCurrent();
  }

  public double getLeftVoltage() {
    return io.getMiddleVoltage();
  }

  public double getRightVelocity() {
    return io.getRightVelocity();
  }

  public double getRightCurrent() {
    return io.getMiddleCurrent();
  }

  public double getRightVoltage() {
    return io.getMiddleVoltage();
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

  public Command setTargetVelolcity(double velocity) {
    return Commands.runOnce(() -> setReference(velocity), this);
  }

  public boolean isReady() {
    return io.isReady();
  }

  public void startShooting() {
    io.startShooting();
  }

  public void stopShooting() {
    io.stopShooting();
  }

  public double getVelocityRadPerSec(int index) {
    return switch (index) {
      case 0 -> io.getLeftVelocity();
      case 1 -> io.getMiddleVelocity();
      case 2 -> io.getRightVelocity();
      default -> io.getMiddleVelocity();
    };
  }

  public void spawnSimulatedBall(int index) {
    io.spawnSimulatedBall(index);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs(this.getName(), inputs);
  }
}
