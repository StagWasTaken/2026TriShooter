package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterReference;
    public boolean readyToShoot;

    public double middleShooterCurrent;
    public double middleShooterVoltage;
    public double middleShooterVelocity;
    public double middleShooterTemp;

    public double leftShooterCurrent;
    public double leftShooterVoltage;
    public double leftShooterVelocity;
    public double leftShooterTemp;

    public double rightShooterCurrent;
    public double rightShooterVoltage;
    public double rightShooterVelocity;
    public double rightShooterTemp;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default double getMiddleCurrent() {
    return 0;
  }

  public default double getMiddleVoltage() {
    return 0;
  }

  public default double getReference() {
    return 0;
  }

  public default double getMiddleVelocity() {
    return 0;
  }

  public default double getLeftCurrent() {
    return 0;
  }

  public default double getLeftVoltage() {
    return 0;
  }

  public default double getLeftVelocity() {
    return 0;
  }

  public default double getRightCurrent() {
    return 0;
  }

  public default double getRightVoltage() {
    return 0;
  }

  public default double getRightVelocity() {
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
