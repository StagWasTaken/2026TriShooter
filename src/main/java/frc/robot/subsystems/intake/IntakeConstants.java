package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public static final int kIntakeCanId = 12;
  public static final int kIntakeBottomRightanId = 1;

  public static final boolean kInverted = false;
  public static final boolean kSecondaryInverted = false;
  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double kIntake = 4500; // 5500 // rpm
  public static final double kOff = 0;
  public static final double kExtake = -12; // volts

  // Top roller gains
  public static final double kP = 0.001;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.13;
  public static final double kV = 0.00177;

  // Top roller geometry — 1.5in diameter, 2:1 reduction
  public static final double kGearRatio = 2.0;
  public static final double kRollerDiameterInches = 1.5;
  public static final double kRollerCircumferenceFeet = (Math.PI * kRollerDiameterInches) / 12.0;
  public static final double kRPMToFtPerSec = kRollerCircumferenceFeet / (kGearRatio * 60.0);
  public static final double kFtPerSecToRPM = 1.0 / kRPMToFtPerSec;

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

    public static final double kP = 33.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kV = 2.9;
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

    public static final double kStowProfileMaxVel = Units.degreesToRadians(30);
    public static final double kStowProfileMaxAccel = Units.degreesToRadians(30);

    public static final double kMaxVel = Math.toRadians(360);
    public static final double kMaxAccel = Math.toRadians(360);

    public static final double kHome = Units.degreesToRadians(70);
    public static final double kStow = Units.degreesToRadians(100);
    public static final double kExtended = Units.degreesToRadians(170);

    public static final double kStowVolts = -2.5;
    public static final double kSlowStowVolts = -1.5;

    public static final double kPositionTolerance = Math.toRadians(10);

    public static final double kGearRatio = 45;
    // absolute encoder has another 3:1 reduction after it
    public static final double kAbsoluteGearRatio = 1;

    public static final double kDeployVoltage = 3;
    public static final double kHomingVelocityThreshold = 0.005;
  }
}
