package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.CMD_Intake;
import frc.robot.commands.CMD_Shoot;
import frc.robot.utils.constants.FieldConstants;

public class AUTO_DepotOnFly implements Auto {
  private final PathPlannerPath shootDepot;
  private final String startPathName = "IntakeDepotOnFly";
  private boolean isMirrored = false;

  public AUTO_DepotOnFly() {
    try {
      shootDepot = Auto.getPath("ShootDepot", isMirrored);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot, double startDelay) {
    Command shoot =
        new CMD_Shoot(
                robot.drive,
                () -> FieldConstants.getHubPose(),
                robot.conveyor,
                robot.hood,
                robot.intake,
                robot.kicker,
                robot.shooter)
            .asProxy();

    return Commands.sequence(
        setAutoStartPose(startPathName, isMirrored, robot.drive),
        new WaitCommand(startDelay),

        // shoot on the fly for 2 seconds while driving the first path
        followPath(startPathName, isMirrored).deadlineWith(shoot).withTimeout(2),

        // intake while continuing backwards on the same path (no timeout, runs to path end)
        followPath(startPathName, isMirrored)
            .deadlineWith(new CMD_Intake(robot.conveyor, robot.intake)),

        // rest of the auto
        sweepPath(shootDepot, robot, 0.0),
        shootCycle(robot, 5));
  }
}
