package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double hoodCurrent;
    public double hoodSupplyCurrent;
    public double hoodVoltage;
    public double hoodVelocity;
    public double hoodReference;
    public double hoodPos;
    public boolean atReference;
    public double hoodTemp;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs, PowerDistribution pdh) {}

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

  public default double getPosition() {
    return 0;
  }

  public default void setVoltage(double voltage) {}

  public default void setReference(double pos) {}

  public default boolean atReference() {
    return false;
  }

  public default void resetEncoder() {}

  public default void periodic() {}
}
