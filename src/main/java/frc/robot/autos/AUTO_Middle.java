package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class AUTO_Middle implements Auto {
  private final String startPathName = "Middle";
  private boolean isMirrored = false;

  /**
   * @param mirrored Whether to mirror the paths and the starting pose.
   */
  public AUTO_Middle() {
    try {
      // Logic for loading paths based on the mirrored boolean
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
        followPath(startPathName, isMirrored),
        shootCycle(robot, 5));
  }
}
