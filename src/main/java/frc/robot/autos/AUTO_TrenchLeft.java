package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_TrenchLeft implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("SweepHalfMiddle", true, robot.drive),
        // intake and sweep half the middle
        new ParallelCommandGroup(new CMD_Intake(robot.intake), followPath("SweepHalfMiddle", true)),
        // turn off intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        followPath("ShootTrench", true),
        // shoot for 2 seconds and then sweep middle again
        robot.shootClose().withTimeout(2),
        new ParallelCommandGroup(new CMD_Intake(robot.intake), followPath("SweepAgain", true)),
        // turn of intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        followPath("ShootTrenchAgain", true),
        // shoot until auto ends
        robot.shootClose());
  }
}
