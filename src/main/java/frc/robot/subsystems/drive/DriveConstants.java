// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4;
  public static final LinearVelocity CHASSIS_MAX_VELOCITY =
      MetersPerSecond.of(maxSpeedMetersPerSec);
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(23.5);
  public static final double wheelBase = Units.inchesToMeters(23.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(moduleTranslations);

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 62;
  public static final int frontRightDriveCanId = 15;
  public static final int backLeftDriveCanId = 8;
  public static final int backRightDriveCanId = 11;

  public static final int frontLeftTurnCanId = 6;
  public static final int frontRightTurnCanId = 14;
  public static final int backLeftTurnCanId = 9;
  public static final int backRightTurnCanId = 10;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.474);
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (12.0 * 15.0); // MAXSwerve with 12 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.06;
  public static final double driveKv = 0.092;
  public static final double driveKa = 0.0;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;
  public static final double driveSimKa = 0.0136;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.5;
  public static final double turnKd = 0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 50;
  public static final double robotMOI = 3.9;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  // these are needed for maple holomicDriveSystem

  /**
   * numbers that needs to be changed to fit each robot TODO: change these numbers to match your
   * robot
   */
  public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.5;

  public static final Mass ROBOT_MASS = Kilograms.of(50); // robot weight with bumpers

  public static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getNeoVortex(1);

  public static final DCMotor STEER_MOTOR_MODEL = DCMotor.getNeo550(1);

  public static final Distance WHEEL_RADIUS = Inches.of(1.45);

  public static final double DRIVE_GEAR_RATIO = driveMotorReduction;
  public static final double STEER_GEAR_RATIO = turnMotorReduction;

  public static final Distance BUMPER_WIDTH = Inches.of(30.25), BUMPER_LENGTH = Inches.of(30.25);
  // https://unacademy.com/content/upsc/study-material/physics/moment-of-inertia-of-rectangle-section/
  public static final MomentOfInertia ROBOT_MOI =
      KilogramSquareMeters.of(
          ROBOT_MASS.in(Kilograms)
              * (BUMPER_WIDTH.in(Meters) * BUMPER_WIDTH.in(Meters)
                  + BUMPER_LENGTH.in(Meters) * BUMPER_LENGTH.in(Meters))
              / 12.0);

  /* adjust current limit */
  public static final Current DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT = Amps.of(80);
  public static final Current DRIVE_OVER_CURRENT_PROTECTION = Amps.of(65);
  public static final Time DRIVE_OVERHEAT_PROTECTION_TIME = Seconds.of(1);
  public static final Current DRIVE_OVERHEAT_PROTECTION_CURRENT = Amps.of(45);
  public static final Current STEER_CURRENT_LIMIT = Amps.of(15);

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
          .withGyro(COTS.ofNav2X())
          .withBumperSize(BUMPER_LENGTH, BUMPER_WIDTH)
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));

  private static final double GRAVITY_CONSTANT = 9.8;
  public static final Distance DRIVE_BASE_RADIUS = Meters.of(MODULE_TRANSLATIONS[0].getNorm());
  /* force = torque / distance */
  public static final Force MAX_PROPELLING_FORCE =
      NewtonMeters.of(
              DRIVE_MOTOR_MODEL.getTorque(DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT.in(Amps))
                  * DRIVE_GEAR_RATIO)
          .div(WHEEL_RADIUS)
          .times(4);

  /* friction_force = normal_force * coefficient_of_friction */
  public static final LinearAcceleration MAX_FRICTION_ACCELERATION =
      MetersPerSecondPerSecond.of(GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION);

  public static final LinearAcceleration CHASSIS_MAX_ACCELERATION =
      (LinearAcceleration)
          Measure.min(MAX_FRICTION_ACCELERATION, MAX_PROPELLING_FORCE.div(ROBOT_MASS));

  public static final AngularVelocity CHASSIS_MAX_ANGULAR_VELOCITY =
      RadiansPerSecond.of(CHASSIS_MAX_VELOCITY.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));
  public static final AngularAcceleration CHASSIS_MAX_ANGULAR_ACCELERATION =
      RadiansPerSecondPerSecond.of(
          CHASSIS_MAX_ACCELERATION.in(MetersPerSecondPerSecond) / DRIVE_BASE_RADIUS.in(Meters) * 2);
}
