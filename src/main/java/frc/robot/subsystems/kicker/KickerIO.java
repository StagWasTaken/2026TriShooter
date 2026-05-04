package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double kickerCurrent;
    public double kickerSupplyCurrent;
    public double kickerVoltage;
    public double kickerVelocity;
    public double kickerReference;
    public boolean atVelocity;
    public double kickerTemp;

    public double kickerBottomRighturrent;
    public double kickerFollowerTemp;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(KickerIOInputs inputs, PowerDistribution pdh) {}

  public default double getCurrent() {
    return 0;
  }

  public default double getVoltage() {
    return 0;
  }

  public default double getReference() {
    return 0;
  }

  public default double getVelocity() {
    return 0;
  }

  public default void setVoltage(double voltage) {}

  public default void setReference(double velocity) {}

  public default boolean atVelocity() {
    return false;
  }

  public default void periodic() {}
}
