// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.SwervePhysicsSim;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.utils.constants.RobotMode;
import frc.robot.utils.hubcounter.HubShiftUtil;
import java.util.HashMap;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  // Physics simulation engine (only used in sim)
  private SwervePhysicsSim sim = new SwervePhysicsSim();
  Pose2d simPose = sim.getPose2d();

  public enum RobotName {
    COMP_BOT,
    PRAC_BOT,
    DEVL_BOT
  }

  private static final RobotMode JAVA_SIM_MODE = RobotMode.SIM;
  public static final RobotMode CURRENT_ROBOT_MODE = isReal() ? RobotMode.REAL : JAVA_SIM_MODE;
  public static final RobotName CURRENT_ROBOT = RobotName.COMP_BOT;

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  public static final boolean LOG_DETAILS = isSimulation();

  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (CURRENT_ROBOT_MODE) {
      case REAL -> {
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> {
        // Running a physics simulator
        // Log to CodeDirectory/logs if you want to test logging system in a simulation
        // Logger.addDataReceiver(new WPILOGWriter());

        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_replayed"),
                WPILOGWriter.AdvantageScopeOpenBehavior.ALWAYS));
      }
    }
    // Initialize URCL for external devices
    Logger.registerURCL(
        URCL.startExternal(
            new HashMap<Integer, String>() {
              {
                put(DriveConstants.frontLeftDriveCanId, "front Left Drive");
                put(DriveConstants.frontRightDriveCanId, "front Right Drive");
                put(DriveConstants.backLeftDriveCanId, "back Left Drive");
                put(DriveConstants.backRightDriveCanId, "back right Drive");
                put(DriveConstants.frontLeftTurnCanId, "front Left Turn");
                put(DriveConstants.frontRightTurnCanId, "front Right Turn");
                put(DriveConstants.backLeftTurnCanId, "back Left Turn");
                put(DriveConstants.backRightTurnCanId, "back right Turn");
                put(IntakeConstants.kIntakeCanId, "intake leader");
                put(IntakeConstants.kIntakeFollowerCanId, "intake follower");
                put(ExtenderConstants.kIntakeExtenderCanId, "intake extender");
                put(ShooterConstants.kLeftShooterCanId, "left shooter");
                put(ShooterConstants.kMiddleShooterCanId, "middle shooter");
                put(ShooterConstants.kRightShooterCanId, "right shooter");
                put(ConveyorConstants.kConveyorCanId, "conveyor");
                put(KickerConstants.kKickerCanId, "kicker");
                put(HoodConstants.kHoodCanId, "hood");
              }
            }));
    // allow the drivetrain to pass over the bump in simulation mode
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Start AdvantageKit logger
    Logger.start();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.updateTelemetryAndLED();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.intake.setVoltage(IntakeConstants.kOff);
    robotContainer.intake.setExtenderReference(robotContainer.intake.getExtenderReference());
    robotContainer.resetSimulationField();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    HubShiftUtil.initialize();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (autonomousCommand != null) CommandScheduler.getInstance().schedule(autonomousCommand);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) autonomousCommand.cancel();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    HubShiftUtil.initialize();
    robotContainer.intake.setExtenderReference(robotContainer.intake.getExtenderPosition());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  private Command testCommand = Commands.none();

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    testCommand.cancel();
    robotContainer.configureButtonBindings();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    robotContainer.resetSimulationField();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();

    // Step physics using the module states commanded by your Drive subsystem
    sim.step(0.02, robotContainer.drive.getModuleStates());
    sim.syncToRealPose(robotContainer.drive.getPose());

    Logger.recordOutput("Robot/Pose", robotContainer.drive.getPose());
    Logger.recordOutput("Robot/Pose3d", sim.getPose3d());
  }
}
