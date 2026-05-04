package frc.robot.subsystems.drum;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Shootable;
import org.littletonrobotics.junction.Logger;

public class Drum extends SubsystemBase implements Shootable {
  private final DrumIO io;
  private final DrumIOInputsAutoLogged inputs = new DrumIOInputsAutoLogged();
  PowerDistribution pdh;

  public Drum(DrumIO io, PowerDistribution pdh) {
    this.io = io;
    this.pdh = pdh;
  }

  @Override
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public Command runVoltage(double voltage) {
    return Commands.runOnce(() -> setVoltage(voltage));
  }

  @Override
  public void setReference(double velocity) {
    io.setReference(velocity);
  }

  @Override
  public void setPreRev(double velocity) {
    io.setPreRev(velocity);
  }

  @Override
  public Command runPreRev(double velocity) {
    return Commands.runOnce(() -> io.setPreRev(velocity));
  }

  @Override
  public Command runVelocity(double velocity) {
    return Commands.runOnce(() -> setReference(velocity));
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
    return io.getVelocity();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs, pdh);
    io.periodic();
    Logger.processInputs(this.getName(), inputs);

    Robot.batteryLogger.reportCurrentUsage(
        this.getName(),
        false,
        inputs.topLeftSupplyCurrent,
        inputs.topRightSupplyCurrent,
        inputs.bottomLeftSupplyCurrent,
        inputs.bottomRightSupplyCurrent);
  }
}
