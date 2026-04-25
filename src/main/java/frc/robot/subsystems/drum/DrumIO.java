package frc.robot.subsystems.drum;

import org.littletonrobotics.junction.AutoLog;

public interface DrumIO {
  @AutoLog
  public static class DrumIOInputs {
    public double shooterReference;
    public double profiledReference;
    public boolean readyToShoot;

    public double topLeftCurrent;
    public double topLeftVoltage;
    public double topLeftVelocity;
    public double topLeftTemp;

    public double bottomLeftCurrent;
    public double bottomLeftVoltage;
    public double bottomLeftTemp;
    public double bottomLeftVel;

    public double topRightCurrent;
    public double topRightVoltage;
    public double topRightTemp;
    public double topRightVel;

    public double bottomRightCurrent;
    public double bottomRightVoltage;
    public double bottomRightTemp;
    public double bottomRightVel;
  }

  public default void updateInputs(DrumIOInputs inputs) {}

  public default double getReference() {
    return 0;
  }

  public default double getVelocity() {
    return 0;
  }

  public default void setVoltage(double voltage) {}

  public default void setReference(double velocity) {}

  public default void setPreRev(double velocity) {}

  public default boolean isReady() {
    return false;
  }

  public default void startShooting() {}

  public default void stopShooting() {}

  public default void periodic() {}
}
