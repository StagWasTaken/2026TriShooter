package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.utils.constants.FieldConstants;

public class AUTO_FollowerSweepAndShoot implements Auto {
  private final PathPlannerPath sweepMiddle;
  private final String startPathName = "FollowerSweepAndShoot";
  private boolean isMirrored;

  /**
   * @param mirrored Whether to mirror the paths and the starting pose.
   */
  public AUTO_FollowerSweepAndShoot(boolean mirrored) {
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
        new WaitCommand(startDelay),
        sweepPath(sweepMiddle, robot, 4.0),
        shootCycle(robot, () -> FieldConstants.getHubPose(), 5));
  }
}
