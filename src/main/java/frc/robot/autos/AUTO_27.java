package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AUTO_27 implements Auto {
  private final PathPlannerPath sweepMiddle;
  private final PathPlannerPath sweepAgain;
  private final String startPathName = "SweepMiddle27";
  private final boolean isMirrored;

  /**
   * @param mirrored Whether to mirror the paths and the starting pose.
   */
  public AUTO_27(boolean mirrored) {
    this.isMirrored = mirrored;
    try {
      // Logic for loading paths based on the mirrored boolean
      sweepMiddle = Auto.getPath(startPathName, isMirrored);
      sweepAgain = Auto.getPath("ShootAndSweepAgain27", isMirrored);
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
        shootCycle(robot, 2.5),
        sweepPath(sweepAgain, robot, 7),
        shootCycle(robot, 5));
  }
}
