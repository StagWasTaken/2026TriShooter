package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeCurrent;
    public double intakeVoltage;
    public double intakeVelocity;
    public double intakeReference;
    public double intakePosition;
    public double intakeTemp;

    public double extenderCurrent;
    public double extenderVoltage;
    public double extenderVelocity;
    public double extenderReference;
    public double extenderPosition;
    public double extenderProfilePositionSetpoint;
    public double extenderProfileVelocitySetpoint;
    public boolean extenderInPosition;
    public double extenderTemp;

    public double intakeFollowerTemp;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

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

  public default void setReference(double velocity) {}

  public default double getExtenderCurrent() {
    return 0;
  }

  public default double getExtenderVoltage() {
    return 0;
  }

  public default double getExtenderReference() {
    return 0;
  }

  public default double getExtenderVelocity() {
    return 0;
  }

  public default double getExtenderPosition() {
    return 0;
  }

  public default boolean getExtenderInPosition() {
    return false;
  }

  public default void setExtenderVoltage(double voltage) {}

  public default void setExtenderReference(double velocity) {}

  public default void setExtenderLowCurrentMode(boolean lowCurrentMode) {}

  public default void periodic() {}
}
