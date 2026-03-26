package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.CMD_Extend;
import frc.robot.commands.CMD_Intake;
import frc.robot.commands.CMD_Shoot;

public class AUTO_DoubleSweepOpponentSideLeft implements Auto {
  private final PathPlannerPath sweepHalfMiddle;
  private final PathPlannerPath sweepAgain;

  public AUTO_DoubleSweepOpponentSideLeft() {
    try {
      sweepHalfMiddle = Auto.getPath("SweepMiddle", true);
      sweepAgain = Auto.getPath("SweepAgainOpponentSide", true);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot) {
    return Commands.sequence(
        setAutoStartPose("SweepMiddle", true, robot.drive),
        new ParallelCommandGroup(
            new CMD_Intake(robot.conveyor, robot.intake), AutoBuilder.followPath(sweepHalfMiddle)),
        robot.shooter.setTargetVelolcity(Math.toRadians(21000)),
        new CMD_Extend(robot.conveyor, robot.intake),
        new CMD_Shoot(
                robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter)
            .withTimeout(3),
        robot.shooter.setTargetVelolcity(Math.toRadians(21000)),
        new ParallelCommandGroup(
            new CMD_Intake(robot.conveyor, robot.intake), AutoBuilder.followPath(sweepAgain)),
        new CMD_Shoot(
            robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter));
  }
}
