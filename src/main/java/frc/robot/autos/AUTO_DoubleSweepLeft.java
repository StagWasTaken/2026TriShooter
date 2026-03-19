package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_DoubleSweepLeft implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("SweepHalfMiddle", true, robot.drive),
        // intake and sweep half the middle
        new ParallelCommandGroup(new CMD_Intake(robot.intake), followPath("SweepHalfMiddle", true)),
        // turn off intake and run back to our side to shoot
        robot.shooter.setTargetVelolcity(Math.toRadians(21000)),
        followPath("ShootTrench", true),
        new CMD_Extend(robot.intake),
        new CMD_Shoot(
                robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter)
            .withTimeout(4),
        robot.shooter.setTargetVelolcity(Math.toRadians(18000)),
        new ParallelCommandGroup(new CMD_Intake(robot.intake), followPath("SweepAgain", true)),
        new CMD_Shoot(
            robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter));
  }
}
