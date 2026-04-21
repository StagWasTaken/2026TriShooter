package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AUTO_Depot implements Auto {
  private final PathPlannerPath intakeDepot;
  private final PathPlannerPath shootDepot;
  private final String startPathName = "IntakeDepot";
  private boolean isMirrored = false;

  /**
   * @param mirrored Whether to mirror the paths and the starting pose.
   */
  public AUTO_Depot() {
    try {
      // Logic for loading paths based on the mirrored boolean
      intakeDepot = Auto.getPath(startPathName, isMirrored);
      shootDepot = Auto.getPath("ShootDepot", isMirrored);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths for Sweep27", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot) {
    return Commands.sequence(
        // Passes the boolean through to the pose reset logic
        setAutoStartPose(startPathName, isMirrored, robot.drive),
        sweepPath(
            intakeDepot, robot, Double.POSITIVE_INFINITY), // Don't rev up shooter on this path
        sweepPath(shootDepot, robot, 0.0), // Start revving up shooter immediately
        shootCycle(robot, 5));
  }
}
