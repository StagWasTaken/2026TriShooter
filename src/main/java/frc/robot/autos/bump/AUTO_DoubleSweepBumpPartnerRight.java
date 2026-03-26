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

public class AUTO_DoubleSweepBumpPartnerRight implements Auto {
  private final PathPlannerPath sweepHalfMiddle;
  private final PathPlannerPath sweepAgain;

  public AUTO_DoubleSweepBumpPartnerRight() {
    try {
      sweepHalfMiddle = Auto.getPath("SweepMiddleBump", false);
      sweepAgain = Auto.getPath("SweepAgainPartnerSideBump", false);
    } catch (Exception e) {
      throw new RuntimeException("Failed to preload auto paths", e);
    }
  }

  @Override
  public Command getAutoCommand(RobotContainer robot) {
    return Commands.sequence(
        setAutoStartPose("SweepMiddleBump", false, robot.drive),
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
