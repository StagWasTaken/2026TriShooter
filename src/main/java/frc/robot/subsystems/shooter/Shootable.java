package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Shootable extends Subsystem {
  void setVoltage(double voltage);

  Command runVoltage(double voltage);

  void setReference(double velocity);

  Command runVelocity(double velocity);

  boolean isReady();

  void startShooting();

  void stopShooting();
}
