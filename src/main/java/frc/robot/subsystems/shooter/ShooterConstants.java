package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.util.Units;
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

  public static final double kIdleVolts =
      0.33; // keeps the flywheels slowly spinning when idle to save power on startup

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

    double entryAngle = -90;

    double deltaThetaDegrees = exitAngle - entryAngle;
    double deltaThetaRadians = Math.toRadians(deltaThetaDegrees);

    // Path radius (Wheel radius + half ball thickness)
    double rPath = kFlywheelRadiusMeters + kGapMeters / 2;

    return rPath * deltaThetaRadians;
  }

  public static final double kStartOnTargetVel = Math.toRadians(720); // radians

  // Shooting table: {distance (meters), angle (degrees), velocity (rpm), time of flight (s)}
  public static final double[][] SHOOTING_TABLE = {
    {3.0480, 75.51, 3323.33, 1.532}, // 120in
    {3.5560, 73.42, 3565.00, 1.608}, // 140in
    {4.0640, 71.33, 3806.67, 1.681}, // 160in
    {4.5720, 69.25, 4048.33, 1.751}, // 180in
    {5.0800, 67.16, 4290.00, 1.816}, // 200in
  };

  // Shooting table: {distance (meters), hood (0-1 rotations), velocity (rad/s), time of flight (s)}
  public static final double[][] SHOOTING_TABLE_REAL = {
    {3.0480, 0.433, 342.7827, 1.515}, // 120in
    {3.5560, 0.5, 377.6892, 1.627}, // 140in
    {4.0640, 0.55, 410.15237, 1.690}, // 160in
    {4.5720, 0.6, 418.87902, 1.740}, // 180in
    {5.0800, 0.6, 436.332, 1.817}, // 200in
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

  public static final ShootingParams getShootingParams(double distance) {
    if (distance <= SHOOTING_TABLE_REAL[0][0]) {
      return new ShootingParams(
          SHOOTING_TABLE_REAL[0][1], SHOOTING_TABLE_REAL[0][2], SHOOTING_TABLE_REAL[0][3]);
    }
    if (distance >= SHOOTING_TABLE_REAL[SHOOTING_TABLE_REAL.length - 1][0]) {
      int last = SHOOTING_TABLE_REAL.length - 1;
      return new ShootingParams(
          SHOOTING_TABLE_REAL[last][1], SHOOTING_TABLE_REAL[last][2], SHOOTING_TABLE_REAL[last][3]);
    }

    for (int i = 0; i < SHOOTING_TABLE_REAL.length - 1; i++) {
      if (distance >= SHOOTING_TABLE_REAL[i][0] && distance <= SHOOTING_TABLE_REAL[i + 1][0]) {
        double d0 = SHOOTING_TABLE_REAL[i][0];
        double d1 = SHOOTING_TABLE_REAL[i + 1][0];
        double t = (distance - d0) / (d1 - d0);

        double hood =
            SHOOTING_TABLE_REAL[i][1]
                + t * (SHOOTING_TABLE_REAL[i + 1][1] - SHOOTING_TABLE_REAL[i][1]);
        double velocity =
            SHOOTING_TABLE_REAL[i][2]
                + t * (SHOOTING_TABLE_REAL[i + 1][2] - SHOOTING_TABLE_REAL[i][2]);
        double tof =
            SHOOTING_TABLE_REAL[i][3]
                + t * (SHOOTING_TABLE_REAL[i + 1][3] - SHOOTING_TABLE_REAL[i][3]);

        return new ShootingParams(hood, velocity, tof);
      }
    }

    return new ShootingParams(0.4, 349.0659, 1.515);
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
