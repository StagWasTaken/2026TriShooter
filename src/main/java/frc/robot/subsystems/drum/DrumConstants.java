package frc.robot.subsystems.drum;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.custompids.MapleShooterOptimization;

public class DrumConstants {
  public static final int kTopLeftCanId = 3;
  public static final int kBottomLeftCanId = 4;
  public static final int kTopRightCanId = 17;
  public static final int kBottomRightCanId = 18;

  public static final double kS = 0.22;
  public static final double kV = 0.002185; // 0.00212
  public static final double kP = 0.0025; // 0.000524
  public static final double kD = 0.0;

  public static final double kProfileMaxVel = 4000.0;
  public static final double kProfileMaxAccel = 20000.0;

  public static final double kPSim = 0.0;
  public static final double kDSim = 0.0;

  public static final double kSSim = 0.0;
  public static final double kVSim = 0.0;

  public static final boolean kInverted = false;
  public static final boolean kBottomLeftInverted = false;
  public static final boolean kTopRightInverted = true;
  public static final boolean kBottomRightInverted = true;

  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double kIdleVolts = 0.5;
  public static final double kStowVolts = -1.67;
  public static final double kSlowStowVolts = -1.0;
  public static final double kStartOnTargetVel = 50;

  // sim
  public static final double kGearRatio = 1;
  public static final double kFuelMassKg = Kilograms.convertFrom(0.5, Pounds);
  public static final double kFuelDiameterMeters = Units.inchesToMeters(5.91);
  public static final double kShooterMOI = 0.00176;
  public static final double kFlywheelRadiusMeters = Units.inchesToMeters(2);
  public static final double kGapMeters = Units.inchesToMeters(5.0);
  public static final double kWheelCOF = .75;
  public static final double kNormalForceNewtons = 80;

  public static double getExitDistMeters(double hoodRotations) {
    double exitAngle = 85.0 - (40.0 * hoodRotations);
    double entryAngle = -90;
    double deltaThetaDegrees = exitAngle - entryAngle;
    double deltaThetaRadians = Math.toRadians(deltaThetaDegrees);
    double rPath = kFlywheelRadiusMeters + kGapMeters / 2;
    return rPath * deltaThetaRadians;
  }

  // Shooting table: {distance (meters), angle (degrees), velocity (rpm), time of flight (s)}
  public static final double[][] SHOOTING_TABLE = {
    {2.5400, 74.00, 3000.00, 0.456}, // 100in
    {3.0480, 70.01, 3273.33, 0.481}, // 120in
    {3.3000, 71.00, 3416.67, 0.464}, // 130in
    {3.5560, 68.00, 3606.67, 0.468}, // 140in
    {4.0640, 66.50, 3916.67, 0.468}, // 160in
    {4.5720, 65.00, 4000.00, 0.495}, // 180in
    {5.0800, 65.00, 4166.67, 0.482} // 200in
  };

  public static final double[][] SHOOTING_TABLE_REAL = {
    {1.524, 3.00, 1900.0, 0.456, 1.214}, // 60in
    {2.032, 15.00, 1832.6, 0.456, 1.334}, // 80in
    {2.540, 17.00, 1916.7, 0.456, 1.328}, // 100in
    {2.794, 21.00, 2100.0, 0.456, 1.282}, // 110in
    {3.048, 22.00, 2250.0, 0.481, 1.248}, // 120in
    {3.300, 25.00, 2300.0, 0.464, 1.283}, // 130in
    {3.810, 25.33, 2400.0, 0.468, 1.375}, // 140in
    {4.064, 26.00, 2500.0, 0.468, 1.332}, // 160in
    {4.318, 27.00, 2550.0, 0.468, 1.328}, // 170in
    {5.080, 25.00, 2700.0, 0.468, 1.196} // 200in
  };

  private static double[] extractColumn(int col) {
    double[] result = new double[SHOOTING_TABLE_REAL.length];
    for (int i = 0; i < SHOOTING_TABLE_REAL.length; i++) {
      result[i] = SHOOTING_TABLE_REAL[i][col];
    }
    return result;
  }

  public static final MapleShooterOptimization kShooterOptimization =
      new MapleShooterOptimization(
          "shooterOptimizationDrum",
          extractColumn(0),
          extractColumn(1),
          extractColumn(2),
          extractColumn(3));

  public static final record ShootingParams(
      double hoodReference, double shooterReference, double tofSeconds)
      implements frc.robot.subsystems.shooter.ShootingParams {}

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

    return new ShootingParams(18, 253.1, 0.336);
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
