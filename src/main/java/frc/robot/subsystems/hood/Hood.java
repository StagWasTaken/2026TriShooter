package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final PowerDistribution pdh;
  private final SysIdRoutine sysIdRoutine;

  public Hood(HoodIO io, PowerDistribution pdh) {
    this.io = io;
    this.pdh = pdh;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("/Hood/SysIdState", state.toString())),
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

  public void setReference(double pos) {
    io.setReference(pos);
  }

  public Command setTargetPos(double pos) {
    return Commands.runOnce(() -> setReference(pos), this);
  }

  public double getPosition() {
    return io.getPosition();
  }

  public boolean atReference() {
    return io.atReference();
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs, pdh);
    Logger.processInputs(this.getName(), inputs);

    Robot.batteryLogger.reportCurrentUsage(this.getName(), false, inputs.hoodSupplyCurrent);
  }
}
