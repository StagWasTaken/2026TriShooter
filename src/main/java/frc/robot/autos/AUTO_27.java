package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      sweepAgain = Auto.getPath("ShootAndSweepAgain", isMirrored);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot, double startDelay) {
    return Commands.sequence(
        // Passes the boolean through to the pose reset logic
        setAutoStartPose(startPathName, isMirrored, robot.drive),
        new WaitCommand(startDelay),
        sweepPath(sweepMiddle, robot, 2),
        shootCycle(robot, 3),
        sweepPath(sweepAgain, robot, 4),
        shootCycle(robot, 5));
  }
}
