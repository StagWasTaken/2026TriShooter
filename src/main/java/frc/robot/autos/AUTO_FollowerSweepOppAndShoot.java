package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.utils.constants.FieldConstants;

public class AUTO_FollowerSweepOppAndShoot implements Auto {
  private final PathPlannerPath sweepMiddle;
  private final String startPathName = "FollowerSweepOppAndShoot";
  private boolean isMirrored;

  /**
   * @param mirrored Whether to mirror the paths and the starting pose.
   */
  public AUTO_FollowerSweepOppAndShoot(boolean mirrored) {
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
        sweepPath(sweepMiddle, robot, 4.5),
        shootCycle(
            robot,
            () -> isMirrored ? FieldConstants.LeftPassingTarget : FieldConstants.RightPassingTarget,
            5));
  }
}
