// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot.RobotName;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.conveyor.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.io.*;
import frc.robot.subsystems.drum.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.kicker.*;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.constants.FieldConstants;
import frc.robot.utils.constants.RobotMode;
import frc.robot.utils.custompids.ChassisHeadingController;
import frc.robot.utils.custompids.MapleJoystickDriveInput;
import frc.robot.utils.hubcounter.HubShiftUtil;
import java.util.function.IntSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Shootable shooter; // Shooter or Drum depending on robot
  public final Intake intake;
  public final Conveyor conveyor;
  public final Kicker kicker;
  public final Hood hood;
  public final Vision vision;
  public final LEDStatusLight ledStatusLight;

  public SwerveDriveSimulation driveSimulation = null;
  private final PowerDistribution powerDistribution;

  private final Field2d field = new Field2d();
  public final DriverMap driver = new DriverMap.LeftHandedXbox(0);
  public final DriverMap operator = new DriverMap.LeftHandedXbox(1);

  public Pose2d resetPose;

  private final LoggedDashboardChooser<Auto> autoChooser;
  private final LoggedNetworkNumber hoodRef, shooterRef;
  public final LoggedTunableNumber autoDelay = new LoggedTunableNumber("Auto/delay", 0.0);

  public RobotContainer() {
    powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    switch (Robot.CURRENT_ROBOT_MODE) {
      case REAL:
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {},
                powerDistribution);

        shooter =
            Robot.CURRENT_ROBOT == Robot.RobotName.DRUM_BOT
                ? new Drum(new DrumIOSpark(), powerDistribution)
                : new Shooter(new ShooterIOSpark());

        intake = new Intake(new IntakeIOSpark(), powerDistribution);
        conveyor = new Conveyor(new ConveyorIOSpark(), powerDistribution);
        kicker = new Kicker(new KickerIOSpark(), powerDistribution);
        hood = new Hood(new HoodIOSpark(), powerDistribution);

        if (Robot.CURRENT_ROBOT == RobotName.HYDRA) {
          this.vision =
              new Vision(
                  drive,
                  () -> drive.getMeasuredChassisSpeedsRobotRelative(),
                  new VisionIOPhotonVision(
                      Vision_Constants.camera0Name, Vision_Constants.robotToCamera0),
                  new VisionIOPhotonVision(
                      Vision_Constants.camera1Name, Vision_Constants.robotToCamera1));
        } else {
          this.vision =
              new Vision(
                  drive,
                  () -> drive.getMeasuredChassisSpeedsRobotRelative(),
                  new VisionIOPhotonVision(
                      Vision_Constants.camera0Name, Vision_Constants.robotToCamera0),
                  new VisionIOPhotonVision(
                      Vision_Constants.camera1Name, Vision_Constants.robotToCamera1),
                  //   new VisionIOPhotonVision(
                  //       Vision_Constants.camera2Name, Vision_Constants.robotToCamera2),
                  new VisionIOPhotonVision(
                      Vision_Constants.camera3Name, Vision_Constants.robotToCamera3));
        }

        break;

      case SIM:
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3.5, 4, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose,
                powerDistribution);

        shooter =
            Robot.CURRENT_ROBOT == Robot.RobotName.DRUM_BOT
                ? new Drum(new DrumIOSim(), powerDistribution)
                : new Shooter(new ShooterIOSim());

        intake = new Intake(new IntakeIOSim(driveSimulation), powerDistribution);
        conveyor = new Conveyor(new ConveyorIOSim(), powerDistribution);
        kicker = new Kicker(new KickerIOSim(), powerDistribution);
        hood = new Hood(new HoodIOSim(), powerDistribution);

        vision =
            new Vision(
                drive,
                () -> drive.getMeasuredChassisSpeedsRobotRelative(),
                new VisionIOPhotonVisionSim(
                    Vision_Constants.camera0Name,
                    Vision_Constants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    Vision_Constants.camera1Name,
                    Vision_Constants.robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {},
                new PowerDistribution());

        shooter =
            Robot.CURRENT_ROBOT == Robot.RobotName.DRUM_BOT
                ? new Drum(new DrumIO() {}, new PowerDistribution())
                : new Shooter(new ShooterIO() {});

        intake = new Intake(new IntakeIO() {}, new PowerDistribution());
        conveyor = new Conveyor(new ConveyorIO() {}, new PowerDistribution());
        kicker = new Kicker(new KickerIO() {}, new PowerDistribution());
        hood = new Hood(new HoodIO() {}, new PowerDistribution());

        vision =
            new Vision(
                drive,
                () -> drive.getMeasuredChassisSpeedsRobotRelative(),
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    this.ledStatusLight = new LEDStatusLight(0, 155, true, false);
    hoodRef = new LoggedNetworkNumber("HoodRef", Robot.CURRENT_ROBOT == RobotName.HYDRA ? .2 : 25);
    shooterRef =
        new LoggedNetworkNumber(
            "ShooterRef",
            Robot.CURRENT_ROBOT == RobotName.HYDRA ? Units.degreesToRadians(12000) : 2450);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // main autos
    autoChooser.addOption("27 Left", new AUTO_27(true));
    autoChooser.addOption("27 Right", new AUTO_27(false));

    // 2nd pick autos
    autoChooser.addOption(
        "Follower Preload & Shoot Left", new AUTO_FollowerSweepAndShootPreload(true));
    autoChooser.addOption(
        "Follower Preload & Shoot Right", new AUTO_FollowerSweepAndShootPreload(false));
    autoChooser.addOption("Follow & Shoot Left", new AUTO_FollowerSweepAndShoot(true));
    autoChooser.addOption("Follow & Shoot Right", new AUTO_FollowerSweepAndShoot(false));
    autoChooser.addOption("Depot", new AUTO_Depot());
    autoChooser.addOption("DepotOnFly", new AUTO_DepotOnFly());
    autoChooser.addOption("Middle", new AUTO_Middle());

    // test / utility autos
    autoChooser.addOption("intake test", new AUTO_3MeterTest());

    configureButtonBindings();

    SmartDashboard.putData("Field", field);
  }

  public void configureButtonBindings() {
    final MapleJoystickDriveInput driveInput = driver.getDriveInput();
    IntSupplier pov = () -> -1;
    final JoystickDrive joystickDrive = new JoystickDrive(driveInput, () -> true, pov, drive);
    drive.setDefaultCommand(joystickDrive);

    hood.setDefaultCommand(
        hood.setTargetPos(
            Robot.CURRENT_ROBOT_MODE == RobotMode.REAL
                ? HoodConstants.kMinPos
                : HoodConstants.kMinHoodAngle));

    ledStatusLight.setDefaultCommand(ledStatusLight.showHubStatus());

    final Runnable resetGyro =
        Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
            ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
    driver.resetOdometryButton().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    if (Robot.CURRENT_ROBOT_MODE == RobotMode.REAL) {
      driver
          .scoreButton()
          .whileTrue(
              new CMD_Shoot(
                  drive,
                  driveInput,
                  () -> FieldConstants.getHubPose(),
                  conveyor,
                  hood,
                  intake,
                  kicker,
                  shooter,
                  () -> !operator.intakeButton().getAsBoolean(),
                  () -> operator.scoreButton().getAsBoolean()));

      driver
          .intakeButton()
          .whileTrue(new CMD_Intake(conveyor, intake))
          .onFalse(new CMD_Extend(conveyor, intake, kicker));

      driver
          .leftBumper()
          .whileTrue(
              new CMD_ShootNoVision(
                  conveyor,
                  hood,
                  intake,
                  kicker,
                  shooter,
                  () -> shooterRef.get(),
                  () -> Robot.CURRENT_ROBOT == RobotName.HYDRA ? 0.25 : hoodRef.get(),
                  () -> !operator.intakeButton().getAsBoolean(),
                  () -> operator.scoreButton().getAsBoolean()));

      driver
          .rightBumper()
          .whileTrue(
              new CMD_Pass(
                  drive,
                  driveInput,
                  conveyor,
                  hood,
                  intake,
                  kicker,
                  shooter,
                  () -> !operator.intakeButton().getAsBoolean(),
                  () -> operator.scoreButton().getAsBoolean()));

      driver
          .yButton()
          .whileTrue(
              new CMD_ShootNoVision(
                  conveyor,
                  hood,
                  intake,
                  kicker,
                  shooter,
                  () -> Robot.CURRENT_ROBOT == RobotName.HYDRA ? Math.toRadians(25000) : 4500,
                  () -> Robot.CURRENT_ROBOT == RobotName.HYDRA ? 0.8 : 33,
                  () -> !operator.intakeButton().getAsBoolean(),
                  () -> operator.scoreButton().getAsBoolean()));

      driver
          .xButton()
          .whileTrue(new CMD_Extake(conveyor, intake, kicker))
          .onFalse(new CMD_Extend(conveyor, intake, kicker));

      driver.aButton().onTrue(new CMD_Home(intake));

      driver
          .bButton()
          .whileTrue(new DynamicPivotJoystickDrive(driver.getDriveInput(), true, () -> -1, drive));

      // operator
      operator.aButton().onTrue(shooter.runPreRev(2000));
      operator.bButton().onTrue(new CMD_HomeHood(hood, shooter));
      operator
          .xButton()
          .whileTrue(intake.runVelocity(IntakeConstants.kIntake))
          .onFalse(intake.runVelocity(IntakeConstants.kOff));
      operator
          .povUp()
          .whileTrue(kicker.runVoltage(KickerConstants.kExtake))
          .onFalse(kicker.runVoltage(KickerConstants.kOff));
      operator
          .povDown()
          .whileTrue(
              conveyor
                  .runVoltage(ConveyorConstants.kExtake)
                  .alongWith(intake.runVoltage(IntakeConstants.kExtake)))
          .onFalse(
              conveyor
                  .runVoltage(ConveyorConstants.kOff)
                  .alongWith(intake.runVoltage(IntakeConstants.kOff)));

    } else if (Robot.CURRENT_ROBOT_MODE == RobotMode.SIM) {
      driver.scoreButton().whileTrue(new CMD_ShootFuelSim(drive, driveSimulation, driveInput));
    }
  }

  public Auto getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Robot.CURRENT_ROBOT_MODE != RobotMode.SIM) return;

    if (FieldConstants.getAlliance() == Alliance.Blue) {
      resetPose = new Pose2d(3.5, 4, new Rotation2d());
    } else {
      resetPose = new Pose2d(13, 4, new Rotation2d());
    }

    drive.resetOdometry(resetPose);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Robot.CURRENT_ROBOT_MODE != RobotMode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput("FieldSimulation/Alliance", FieldConstants.getAlliance().toString());
    Logger.recordOutput("FieldSimulation/BlueScore", SimulatedArena.getInstance().getScore(true));
    Logger.recordOutput("FieldSimulation/RedScore", SimulatedArena.getInstance().getScore(false));
  }

  public static boolean motorBrakeEnabled = false;

  public void setMotorBrake(boolean brakeModeEnabled) {
    if (motorBrakeEnabled == brakeModeEnabled) return;
    System.out.println("Set motor brake: " + brakeModeEnabled);
    drive.setMotorBrake(brakeModeEnabled);
    motorBrakeEnabled = brakeModeEnabled;
  }

  public void updateTelemetryAndLED() {
    field.setRobotPose(
        Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
            ? driveSimulation.getSimulatedDriveTrainPose()
            : drive.getPose());
    if (Robot.CURRENT_ROBOT_MODE == RobotMode.SIM)
      field.getObject("Odometry").setPose(drive.getPose());

    Logger.recordOutput("RioVoltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("Match Time", DriverStation.getMatchTime());
    Logger.recordOutput("Hub Active", HubShiftUtil.getOfficialShiftInfo().active());
    Logger.recordOutput(
        "Hub Duration Remaining", HubShiftUtil.getOfficialShiftInfo().remainingTime());
    Logger.recordOutput(
        "ChassisHeadingControllerAtSetpoint", ChassisHeadingController.getInstance().atSetPoint());
    Logger.recordOutput(
        "DistFromHub",
        Units.metersToInches(
            FieldConstants.getHubPose().getDistance(drive.getPose().getTranslation())));
  }
}
