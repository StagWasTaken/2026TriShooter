package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
  @AutoLog
  public static class ConveyorIOInputs {
    public double conveyorCurrent;
    public double conveyorSupplyCurrent;
    public double conveyorVoltage;
    public double conveyorVelocity;
    public double conveyorReference;
    public double conveyorTemp;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ConveyorIOInputs inputs, PowerDistribution pdh) {}

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

  public default void periodic() {}
}
