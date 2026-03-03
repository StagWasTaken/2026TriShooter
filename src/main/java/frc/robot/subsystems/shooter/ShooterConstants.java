package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.utils.CustomPIDs.MapleShooterOptimization;

public class ShooterConstants {
  public static final int kLeftShooterCanId = 3;
  public static final int kMiddleShooterCanId = 4;
  public static final int kRightShooterCanId = 17;

  public static final double kLeftShooterS = 0.13;
  public static final double kMiddleShooterS = 0.12;
  public static final double kRightShooterS = 0.13;

  public static final double kLeftShooterV = 0.0194;
  public static final double kMiddleShooterV = 0.019;
  public static final double kRightShooterV = 0.0202;

  public static final double kLeftShooterP = 0.0165;
  public static final double kMiddleShooterP = 0.0165;
  public static final double kRightShooterP = 0.0165;

  public static final double kLeftShooterD = 0.00075;
  public static final double kMiddleShooterD = 0.00075;
  public static final double kRightShooterD = 0.00075;

  public static final double kPSim = 0.3;
  public static final double kDSim = 0.0;

  public static final double kSSim = 0.0;
  public static final double kVSim = 0.0198425;

  public static final boolean kInverted = true;

  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  // sim
  public static final double kGearRatio = 1;
  public static final double kFuelMassKg = Kilograms.convertFrom(0.5, Pounds);
  public static final double kFuelDiameterMeters = Units.inchesToMeters(5.91);
  public static final double kShooterMOI = 0.00176;
  public static final double kFlywheelRadiusMeters = Units.inchesToMeters(2);
  // distance between hood and flywheel
  public static final double kGapMeters = Units.inchesToMeters(5.0);
  // rough COF between polyurethane foam game piece and TPU
  public static final double kWheelCOF = .75;
  public static final double kNormalForceNewtons = 80; // 130

  public static double getExitDistMeters(double hoodRotations) {
    // Current exit angle in degrees
    double exitAngle = 85.0 - (40.0 * hoodRotations);

    // Assuming entry is fixed at the "bottom" of the arc (e.g., 135 degrees)
    // You'll need to check your CAD for the actual entry transition point
    double entryAngle = -90;

    double deltaThetaDegrees = exitAngle - entryAngle;
    double deltaThetaRadians = Math.toRadians(deltaThetaDegrees);

    // Path radius (Wheel radius + half ball thickness)
    double rPath = kFlywheelRadiusMeters + kGapMeters / 2;

    return rPath * deltaThetaRadians;
  }

  public static final double kStartOnTargetVel = Math.toRadians(720); // radians

  // Shooting table: {distance (meters), angle (degrees), velocity (m/s), time of flight (s)}
  public static final double[][] SHOOTING_TABLE = {
    {1.0, 85.00, 6.02, 0.7608},
    {2.0, 65.43, 6.43, 0.7482},
    {3.0, 56.90, 7.04, 0.7641},
    {4.0, 58.75, 7.86, 0.9812},
    {5.0, 48.88, 8.47, 0.8978},
    {6.0, 42.88, 9.29, 0.8818},
    {7.0, 41.60, 9.90, 0.9457},
    {8.0, 40.16, 10.51, 0.9960},
    {9.0, 38.67, 11.12, 1.0365},
    {10.0, 37.20, 11.73, 1.0699}
  };

  // Extract columns for MapleShooterOptimization
  private static double[] extractColumn(int col) {
    double[] result = new double[SHOOTING_TABLE.length];
    for (int i = 0; i < SHOOTING_TABLE.length; i++) {
      result[i] = SHOOTING_TABLE[i][col];
    }
    return result;
  }

  public static final MapleShooterOptimization kShooterOptimization =
      new MapleShooterOptimization(
          "shooterOptimization",
          extractColumn(0), // distance, meters
          extractColumn(1), // hood angle, degrees
          extractColumn(2), // shooter velocity, m/s
          extractColumn(3) // time of flight, seconds
          );

  public static final record ShootingParams(
      double hoodReference, double shooterReference, double tofSeconds) {}

  public static final ShootingParams getShootingParams(double distanceMeters) {
    // linear regressions calculated from desmos
    double hoodAngle = (0.123023 * distanceMeters) - 0.18125;
    double shooterVelRad = (44.19548 * distanceMeters) + 186.99956;
    shooterVelRad = MathUtil.clamp(shooterVelRad, 350, 450);

    // 11m/s^2 gravity to match maple sim
    double g = 11;
    double targetHeightM = Units.feetToMeters(6.0);

    // convert shooter flywheel angular speed in rad/s to linear speed meter/s, with a roughly
    // 43.5% speed transfer to projectile
    double projVelMps = shooterVelRad * Units.inchesToMeters(2) * 0.435;
    // convert from 0 = 90 degress from horizontal to 0 = parralel to floor
    double launchAngleRad =
        HoodConstants.kMaxHoodAngle - (HoodConstants.kHoodAngleDelta * hoodAngle);
    // hood angle is 1 rotation of the motor = 45 degrees of hood travel
    double vy = (projVelMps) * Math.sin(launchAngleRad);
    // shooter is mounted 21 inches from the floor, target height is 6 feet off the floor
    double dy = Units.inchesToMeters(21) - targetHeightM;

    double tofSeconds = (vy + Math.sqrt(vy * vy + 2 * g * dy)) / g;

    return new ShootingParams(launchAngleRad, shooterVelRad, tofSeconds);
  }

  public static final ShootingParams getSimShootingParams(double distance) {
    if (distance <= SHOOTING_TABLE[0][0]) {
      return new ShootingParams(
          Units.degreesToRadians(SHOOTING_TABLE[0][1]), SHOOTING_TABLE[0][2], SHOOTING_TABLE[0][3]);
    }
    if (distance >= SHOOTING_TABLE[SHOOTING_TABLE.length - 1][0]) {
      int last = SHOOTING_TABLE.length - 1;
      return new ShootingParams(
          Units.degreesToRadians(SHOOTING_TABLE[last][1]),
          SHOOTING_TABLE[last][2],
          SHOOTING_TABLE[last][3]);
    }

    for (int i = 0; i < SHOOTING_TABLE.length - 1; i++) {
      if (distance >= SHOOTING_TABLE[i][0] && distance <= SHOOTING_TABLE[i + 1][0]) {
        double d0 = SHOOTING_TABLE[i][0];
        double d1 = SHOOTING_TABLE[i + 1][0];
        double t = (distance - d0) / (d1 - d0);

        double angle = SHOOTING_TABLE[i][1] + t * (SHOOTING_TABLE[i + 1][1] - SHOOTING_TABLE[i][1]);
        double velocity =
            SHOOTING_TABLE[i][2] + t * (SHOOTING_TABLE[i + 1][2] - SHOOTING_TABLE[i][2]);
        double tof = SHOOTING_TABLE[i][3] + t * (SHOOTING_TABLE[i + 1][3] - SHOOTING_TABLE[i][3]);

        return new ShootingParams(Units.degreesToRadians(angle), velocity, tof);
      }
    }

    return new ShootingParams(Units.degreesToRadians(75), 7.0, 1.0);
  }
}
