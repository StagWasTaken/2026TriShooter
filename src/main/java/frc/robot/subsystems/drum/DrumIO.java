package frc.robot.subsystems.drum;

import org.littletonrobotics.junction.AutoLog;

public interface DrumIO {
  @AutoLog
  public static class DrumIOInputs {
    public double shooterReference;
    public boolean readyToShoot;

    // Leader — used for control
    public double leaderCurrent;
    public double leaderVoltage;
    public double leaderVelocity;
    public double leaderTemp;

    // Followers — logged for diagnostics only
    public double followerACurrent;
    public double followerATemp;
    public double followerAVel;

    public double followerBCurrent;
    public double followerBTemp;
    public double followerBVel;

    public double followerCCurrent;
    public double followerCTemp;
    public double followerCVel;
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

  public default boolean isReady() {
    return false;
  }

  public default void startShooting() {}

  public default void stopShooting() {}

  public default void periodic() {}
}
