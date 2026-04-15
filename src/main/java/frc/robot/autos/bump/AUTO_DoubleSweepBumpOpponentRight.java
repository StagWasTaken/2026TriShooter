package frc.robot.autos.bump;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.*;
import frc.robot.utils.constants.FieldConstants;

public class AUTO_DoubleSweepBumpOpponentRight implements Auto {
  private final PathPlannerPath sweepHalfMiddle;
  private final PathPlannerPath sweepAgain;

  public AUTO_DoubleSweepBumpOpponentRight() {
    try {
      sweepHalfMiddle = Auto.getPath("SweepMiddleBump", false);
      sweepAgain = Auto.getPath("SweepAgainOpponentSideBump", false);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  private Command sweepPath(PathPlannerPath path, RobotContainer robot, double spinupDelay) {
    return new ParallelCommandGroup(
        AutoBuilder.followPath(path),
        new CMD_Intake(robot.intake),
        new SequentialCommandGroup(new WaitCommand(spinupDelay), robot.shooter.runVelocity(2000)));
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
        setAutoStartPose("SweepMiddleBump", false, robot.drive),
        sweepPath(sweepHalfMiddle, robot, 5),
        shootCycle(robot, 3.5),
        sweepPath(sweepAgain, robot, 7),
        shootCycle(robot, 5));
  }
}
