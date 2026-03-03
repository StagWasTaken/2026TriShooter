package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_Trench implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot, boolean mirrored)
      throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("SweepHalfMiddle", mirrored, robot.drive),
        // intake and sweep half the middle
        new ParallelCommandGroup(
            new CMD_Intake(robot.intake), followPath("SweepHalfMiddle", mirrored)),
        // turn off intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        followPath("ShootTrench", mirrored),
        // shoot for 2 seconds and then sweep middle again
        robot.shootClose().withTimeout(2),
        new ParallelCommandGroup(new CMD_Intake(robot.intake), followPath("SweepAgain", mirrored)),
        // turn of intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        followPath("ShootTrench", mirrored),
        // shoot until auto ends
        robot.shootClose());
  }
}
