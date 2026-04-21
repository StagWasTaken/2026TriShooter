package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int kIntakeCanId = 12;
  public static final int kIntakeBottomRightanId = 1;

  public static final boolean kInverted = false;
  public static final boolean kSecondaryInverted = false;
  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double kIntake = 5500; // rpm
  public static final double kOff = 0;
  public static final double kExtake = -12; // volts

  public static final double kProfileMaxVel = 10000; // rpm
  public static final double kProfileMaxAccel = 25000; // rpm

  // Top roller gains
  public static final double kP = 0.001;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.13;
  public static final double kV = 0.00177;

  // Bottom roller gains
  public static final double kPBottom = 0.0;
  public static final double kIBottom = 0.0;
  public static final double kDBottom = 0.0;

  public static final double kSBottom = 0.13;
  public static final double kVBottom = 0.00177;

  // Top roller geometry — 1.5in diameter, 2:1 reduction
  public static final double kGearRatio = 2.0;
  public static final double kRollerDiameterInches = 1.5;
  public static final double kRollerCircumferenceFeet = (Math.PI * kRollerDiameterInches) / 12.0;
  public static final double kRPMToFtPerSec = kRollerCircumferenceFeet / (kGearRatio * 60.0);
  public static final double kFtPerSecToRPM = 1.0 / kRPMToFtPerSec;

  // Bottom roller geometry — 1.125in diameter, 2:1 reduction
  public static final double kBottomGearRatio = 2.0;
  public static final double kBottomRollerDiameterInches = 1.125;
  public static final double kBottomRollerCircumferenceFeet =
      (Math.PI * kBottomRollerDiameterInches) / 12.0;
  public static final double kBottomRPMToFtPerSec =
      kBottomRollerCircumferenceFeet / (kBottomGearRatio * 60.0);
  public static final double kBottomFtPerSecToRPM = 1.0 / kBottomRPMToFtPerSec;

  // sim
  public static final double kPSim = 0.0;
  public static final double kISim = 0.0;
  public static final double kDSim = 0.0;

  public static final double kSSim = 0.0;
  public static final double kVSim = 0.0;
  public static final double kGSim = 0.0;
  public static final double kASim = 0.0;

  public static final class ExtenderConstants {
    public static final int kIntakeExtenderCanId = 7;

    public static final double kP = 0.0;
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
    public static final double kExtended = Math.toRadians(315);

    public static final double kPositionTolerance = Math.toRadians(10);

    public static final double kGearRatio = 81;
    // absolute encoder has another 3:1 reduction after it
    public static final double kAbsoluteGearRatio = 3;

    public static final double kDeployVoltage = 5;
    public static final double kHomingVelocityThreshold = 100;
  }
}
