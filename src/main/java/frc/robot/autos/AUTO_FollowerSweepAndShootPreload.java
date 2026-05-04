package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AUTO_FollowerSweepAndShootPreload implements Auto {
  private final PathPlannerPath sweepMiddle;
  private final String startPathName = "FollowerSweepAndShootPreload";
  private boolean isMirrored;

  /**
   * @param mirrored Whether to mirror the paths and the starting pose.
   */
  public AUTO_FollowerSweepAndShootPreload(boolean mirrored) {
    this.isMirrored = mirrored;
    try {
      // Logic for loading paths based on the mirrored boolean
      sweepMiddle = Auto.getPath(startPathName, isMirrored);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot, double startDelay) {
    return Commands.sequence(
        setAutoStartPose(startPathName, isMirrored, robot.drive),
        shootCycle(robot, startDelay), // shoot for the delay timer and then run
        sweepPath(sweepMiddle, robot, 4.0),
        shootCycle(robot, 5));
  }
}
