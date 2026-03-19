package frc.robot.subsystems.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final int kHoodCanId = 2;

  public static final double kP = 5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kV = 0.0;
  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kA = 0.0;

  public static final double kPSim = 20;
  public static final double kDSim = 0.0;

  public static final boolean kInverted = false;
  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double kMinPos = 0.025;
  public static final double kMaxPos = 0.95;
  public static final double kTolerance = Math.toRadians(2);

  public static final double kMaxVel = 1;
  public static final double kMaxAccel = 3;

  public static final double kGearRatio = 25;

  // sim
  public static final double kHoodMOI = 0.0145;
  public static final double kMinHoodAngle = Math.toRadians(53);
  public static final double kMaxHoodAngle = Math.toRadians(85);
  public static final double kHoodAngleDelta = kMaxHoodAngle - kMinHoodAngle;
  public static final double kStartHoodAngle = kMinHoodAngle;

  // 5 inch gap between flywheel and hood, 2 inch radius flywheel, and 1 inch thick hood
  public static final double kHoodRadius = Units.inchesToMeters(7.5);
}
