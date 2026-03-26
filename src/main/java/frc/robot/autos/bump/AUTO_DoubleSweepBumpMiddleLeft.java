package frc.robot.autos.bump;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.CMD_Extend;
import frc.robot.commands.CMD_Intake;
import frc.robot.commands.CMD_Shoot;

public class AUTO_DoubleSweepBumpMiddleLeft implements Auto {
  private final PathPlannerPath sweepHalfMiddle;
  private final PathPlannerPath sweepAgain;

  public AUTO_DoubleSweepBumpMiddleLeft() {
    try {
      sweepHalfMiddle = Auto.getPath("SweepMiddleBump", true);
      sweepAgain = Auto.getPath("SweepAgainMiddleBump", true);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot) {
    return Commands.sequence(
        setAutoStartPose("SweepMiddleBump", true, robot.drive),
        new ParallelCommandGroup(
            new CMD_Intake(robot.conveyor, robot.intake), AutoBuilder.followPath(sweepHalfMiddle)),
        new CMD_Extend(robot.conveyor, robot.intake),
        new CMD_Shoot(
                robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter)
            .withTimeout(3.5),
        new ParallelCommandGroup(
            new CMD_Intake(robot.conveyor, robot.intake), AutoBuilder.followPath(sweepAgain)),
        new CMD_Extend(robot.conveyor, robot.intake),
        new CMD_Shoot(
            robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter));
  }
}
