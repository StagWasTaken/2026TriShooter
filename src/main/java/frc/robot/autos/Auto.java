package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.constants.FieldConstants;
import java.io.IOException;
import java.util.function.Supplier;
import org.ironmaple.utils.FieldMirroringUtils;
import org.json.simple.parser.ParseException;

public interface Auto {
  /** Tunable delay to avoid collisions with partners at the start of Auto. */
  static final LoggedTunableNumber autoStartDelay =
      new LoggedTunableNumber("Auto/StartDelaySeconds", 0.0);

  Command getAutoCommand(RobotContainer robot, double startDelay)
      throws IOException, ParseException;

  default Command shootCycle(
      RobotContainer robot, Supplier<Translation2d> targetSupplier, double timeout) {
    return Commands.sequence(
        new CMD_Shoot(
                robot.drive,
                () -> targetSupplier.get(),
                robot.conveyor,
                robot.hood,
                robot.intake,
                robot.kicker,
                robot.shooter)
            .withTimeout(timeout));
  }

  default Command shootCycle(RobotContainer robot, double timeout) {
    return shootCycle(robot, () -> FieldConstants.getHubPose(), timeout);
  }

  default Command sweepPath(PathPlannerPath path, RobotContainer robot, double spinupDelay) {
    return AutoBuilder.followPath(path)
        .deadlineWith(
            new CMD_Intake(robot.conveyor, robot.intake),
            Commands.sequence(new WaitCommand(spinupDelay), robot.shooter.runPreRev(2000)));
  }

  static Auto none() {
    return new Auto() {
      @Override
      public Command getAutoCommand(RobotContainer robot, double startDelay) {
        return Commands.none();
      }
    };
  }

  default Command setAutoStartPose(String pathName, Boolean mirrored, Drive drive) {
    final PathPlannerPath finalPath;
    try {
      PathPlannerPath loadedPath = PathPlannerPath.fromPathFile(pathName);
      if (mirrored) {
        loadedPath = loadedPath.mirrorPath();
      }
      if (DriverStation.getAlliance().isPresent() && FieldConstants.getAlliance() == Alliance.Red) {
        loadedPath = loadedPath.flipPath();
      }
      finalPath = loadedPath;
    } catch (Exception e) {
      DriverStation.reportError("Error: failed to load path: " + pathName, e.getStackTrace());
      return Commands.none();
    }
    return Commands.runOnce(() -> drive.resetOdometry(finalPath.getStartingHolonomicPose().get()));
  }

  static PathPlannerPath getPath(String name, boolean mirror) throws IOException, ParseException {
    PathPlannerPath path = PathPlannerPath.fromPathFile(name);
    return mirror ? path.mirrorPath() : path;
  }

  static Pose2d flipLeftRight(Pose2d pose) {
    return new Pose2d(
        pose.getX(),
        FieldMirroringUtils.FIELD_HEIGHT - pose.getY(),
        pose.getRotation().unaryMinus());
  }

  default Command followPath(String pathName, boolean mirrored) {
    PathPlannerPath path;
    try {
      path = getPath(pathName, mirrored);
    } catch (Exception e) {
      DriverStation.reportError("Error: failed to load path: " + pathName, e.getStackTrace());
      return Commands.none();
    }
    return AutoBuilder.followPath(path);
  }
}
