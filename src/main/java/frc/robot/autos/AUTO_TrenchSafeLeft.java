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

public class AUTO_TrenchSafeLeft implements Auto {
  private final PathPlannerPath sweepHalfMiddle;
  private final PathPlannerPath sweepAgain;

  public AUTO_TrenchSafeLeft() {
    try {
      sweepHalfMiddle = Auto.getPath("SweepMiddleSafe", true);
      sweepAgain = Auto.getPath("SweepAgain", true);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot) {
    return Commands.sequence(
        setAutoStartPose("SweepMiddleSafe", true, robot.drive),
        new ParallelCommandGroup(
            new CMD_Intake(robot.intake), AutoBuilder.followPath(sweepHalfMiddle)),
        robot.shooter.setTargetVelolcity(Math.toRadians(21000)),
        new CMD_Extend(robot.intake),
        new CMD_Shoot(
                robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter)
            .withTimeout(3),
        new ParallelCommandGroup(new CMD_Intake(robot.intake), AutoBuilder.followPath(sweepAgain)),
        robot.shooter.setTargetVelolcity(Math.toRadians(21000)),
        new CMD_Extend(robot.intake),
        new CMD_Shoot(
                robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter)
            .withTimeout(3));
  }
}
