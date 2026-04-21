package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AUTO_FollowerSweepOppAndPass implements Auto {
  private final PathPlannerPath sweepMiddle;
  private final String startPathName = "FollowerSweepOppAndPass";
  private boolean isMirrored;

  /**
   * @param mirrored Whether to mirror the paths and the starting pose.
   */
  public AUTO_FollowerSweepOppAndPass(boolean mirrored) {
    this.isMirrored = mirrored;
    try {
      // Logic for loading paths based on the mirrored boolean
      sweepMiddle = Auto.getPath(startPathName, isMirrored);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths for Sweep27", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot) {
    return Commands.sequence(
        // Passes the boolean through to the pose reset logic
        setAutoStartPose(startPathName, isMirrored, robot.drive),
        sweepPath(sweepMiddle, robot, 6.33),
        shootCycle(robot, 5));
  }
}
