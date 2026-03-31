package frc.robot.autos.trench;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.*;
import frc.robot.utils.constants.FieldConstants;

public class AUTO_DoubleSweepPartnerSideLeft implements Auto {
  private final PathPlannerPath sweepHalfMiddle;
  private final PathPlannerPath sweepAgain;

  public AUTO_DoubleSweepPartnerSideLeft() {
    try {
      sweepHalfMiddle = Auto.getPath("SweepMiddle", true);
      sweepAgain = Auto.getPath("SweepAgainPartnerSide", true);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  private Command sweepPath(PathPlannerPath path, RobotContainer robot) {
    return new ParallelCommandGroup(
        AutoBuilder.followPath(path),
        new CMD_Intake(robot.conveyor, robot.intake),
        Commands.sequence(Commands.waitSeconds(3), robot.shooter.runVoltage(6)));
  }

  private Command shootCycle(RobotContainer robot, double timeout) {
    return Commands.sequence(
        new CMD_Extend(robot.conveyor, robot.intake),
        new CMD_Shoot(
                robot.drive,
                () -> FieldConstants.getHubPose(),
                robot.conveyor,
                robot.hood,
                robot.intake,
                robot.kicker,
                robot.shooter)
            .withTimeout(timeout));
  }

  @Override
  public Command getAutoCommand(RobotContainer robot) {
    return Commands.sequence(
        setAutoStartPose("SweepMiddle", true, robot.drive),
        sweepPath(sweepHalfMiddle, robot),
        shootCycle(robot, 3),
        sweepPath(sweepAgain, robot),
        shootCycle(robot, 3));
  }
}
