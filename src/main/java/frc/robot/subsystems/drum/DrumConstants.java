package frc.robot.subsystems.drum;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.MapleInterpolationTable;
import frc.robot.utils.custompids.MapleShooterOptimization;

public class DrumConstants {
  public static final int kTopLeftCanId = 3;
  public static final int kBottomLeftCanId = 4;
  public static final int kTopRightCanId = 17;
  public static final int kBottomRightCanId = 18;

  public static final double kS = 0.22;
  public static final double kV = 0.002185;
  public static final double kP = 0.0025;
  public static final double kD = 0.0;

  public static final double kProfileMaxVel = 4000.0;
  public static final double kProfileMaxAccel = 5000.0;

  public static final double kPreRevMaxVel = 1000.0;
  public static final double kPreRevMaxAccel = 1000.0;

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

  // Sim shooting table: {distance (meters), angle (degrees), velocity (rpm), tof (s)}
  public static final double[][] SHOOTING_TABLE = {
    {2.5400, 74.00, 3000.00, 0.456}, // 100in
    {3.0480, 70.01, 3273.33, 0.481}, // 120in
    {3.3000, 71.00, 3416.67, 0.464}, // 130in
    {3.5560, 68.00, 3606.67, 0.468}, // 140in
    {4.0640, 66.50, 3916.67, 0.468}, // 160in
    {4.5720, 65.00, 4000.00, 0.495}, // 180in
    {5.0800, 65.00, 4166.67, 0.482}, // 200in
  };

  // Real shooting table: {distance (meters), hood (degrees), velocity (rpm), tof (s)}
  public static final double[][] SHOOTING_TABLE_REAL = {
    {1.524, 3.00, 1900.0, 0.456}, // 60in
    {2.032, 16.00, 1800.0, 0.456}, // 80in
    {2.540, 16.00, 2200.0, 0.456}, // 100in
    {2.794, 22.00, 2200.0, 0.456}, // 110in
    {3.048, 23.00, 2300.0, 0.481}, // 120in
    {3.300, 25.00, 2400.0, 0.464}, // 130in
    {3.429, 25.00, 2450.0, 0.464}, // 135in
    {3.810, 25.00, 2500.0, 0.468}, // 140in
    {4.064, 27.00, 2600.0, 0.468}, // 160in
    {4.318, 28.00, 2650.0, 0.468}, // 170in
    {5.080, 26.00, 2800.0, 0.468}, // 200in
  };

  // Passing table: {distance (meters), hood (degrees), velocity (rpm), tof (s)}
  public static final double[][] PASSING_TABLE_REAL = {
    {6.350, 45.0, 2000.0, 0.5}, // 250in
    {7.620, 50.0, 2600.0, 0.75}, // 300in
    {8.636, 50.0, 2800.0, 1.0}, // 340in
  };

  private static double[] extractColumn(double[][] table, int col) {
    double[] result = new double[table.length];
    for (int i = 0; i < table.length; i++) {
      result[i] = table[i][col];
    }
    return result;
  }

  public static final MapleShooterOptimization kShooterOptimization =
      new MapleShooterOptimization(
          "shooterOptimizationDrum",
          extractColumn(SHOOTING_TABLE_REAL, 0),
          extractColumn(SHOOTING_TABLE_REAL, 1),
          extractColumn(SHOOTING_TABLE_REAL, 2),
          extractColumn(SHOOTING_TABLE_REAL, 3));

  public static final MapleInterpolationTable SHOOTING_TABLE_INTERP =
      new MapleInterpolationTable(
          "ShootingTableReal",
          new MapleInterpolationTable.Variable("distance", extractColumn(SHOOTING_TABLE_REAL, 0)),
          new MapleInterpolationTable.Variable("hood", extractColumn(SHOOTING_TABLE_REAL, 1)),
          new MapleInterpolationTable.Variable("velocity", extractColumn(SHOOTING_TABLE_REAL, 2)),
          new MapleInterpolationTable.Variable("tof", extractColumn(SHOOTING_TABLE_REAL, 3)));

  public static final MapleInterpolationTable PASSING_TABLE_INTERP =
      new MapleInterpolationTable(
          "PassingTableReal",
          new MapleInterpolationTable.Variable("distance", extractColumn(PASSING_TABLE_REAL, 0)),
          new MapleInterpolationTable.Variable("hood", extractColumn(PASSING_TABLE_REAL, 1)),
          new MapleInterpolationTable.Variable("velocity", extractColumn(PASSING_TABLE_REAL, 2)),
          new MapleInterpolationTable.Variable("tof", extractColumn(PASSING_TABLE_REAL, 3)));

  public static final record ShootingParams(
      double hoodReference, double shooterReference, double tofSeconds)
      implements frc.robot.subsystems.shooter.ShootingParams {}

  public static ShootingParams getShootingParams(double distance) {
    return new ShootingParams(
        SHOOTING_TABLE_INTERP.interpolateVariableWithLimit("hood", distance),
        SHOOTING_TABLE_INTERP.interpolateVariableWithLimit("velocity", distance),
        SHOOTING_TABLE_INTERP.interpolateVariableWithLimit("tof", distance));
  }

  public static ShootingParams getPassingParams(double distance) {
    return new ShootingParams(
        PASSING_TABLE_INTERP.interpolateVariableWithLimit("hood", distance),
        PASSING_TABLE_INTERP.interpolateVariableWithLimit("velocity", distance),
        PASSING_TABLE_INTERP.interpolateVariableWithLimit("tof", distance));
  }

  public static ShootingParams getSimShootingParams(double distance) {
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
