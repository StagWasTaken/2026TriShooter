package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements Shootable {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public Command runVoltage(double voltage) {
    return Commands.runOnce(() -> setVoltage(voltage), this);
  }

  @Override
  public void setReference(double velocity) {
    io.setReference(velocity);
  }

  @Override
  public Command runVelocity(double velocity) {
    return Commands.runOnce(() -> setReference(velocity), this);
  }

  @Override
  public boolean isReady() {
    return io.isReady();
  }

  @Override
  public void startShooting() {
    io.startShooting();
  }

  @Override
  public void stopShooting() {
    io.stopShooting();
  }

  @Override
  public double getVelocity() {
    return io.getLeftVelocity();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs(this.getName(), inputs);
  }
}
