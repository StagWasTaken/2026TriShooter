package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int kIntakeCanId = 12;
  public static final int kIntakeFollowerCanId = 1;

  public static final double kP = 0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kV = 0.0;
  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kA = 0;

  public static final double kPSim = 0.0;
  public static final double kISim = 0.0;
  public static final double kDSim = 0.0;

  public static final double kSSim = 0.0;
  public static final double kVSim = 0.0;
  public static final double kGSim = 0.0;
  public static final double kASim = 0.0;

  public static final boolean kInverted = false;
  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double kOn = 10;
  public static final double kOff = 0;

  public static final double kGearRatio = 1;

  public static final class ExtenderConstants {
    public static final int kIntakeExtenderCanId = 7;

    public static final double kP = 3;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kA = 0.0;

    public static final double kPSim = 0.25;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;

    public static final double kSSim = 0.02;
    public static final double kVSim = 1.725;
    public static final double kGSim = 0.825;
    public static final double kASim = 0.0;

    public static final boolean kInverted = false;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kHome = Math.toRadians(70);
    public static final double kStow = Math.toRadians(120);
    public static final double kExtended = Math.toRadians(270);

    public static final double kMaxVel = Math.toRadians(43200);
    public static final double kMaxAccel = Math.toRadians(86400);

    public static final double kPositionTolerance = Math.toRadians(50);

    public static final double kGearRatio = 81;
    // absolute encoder has another 3:1 reduction after it
    public static final double kAbsoluteGearRatio = 3;
  }
}
