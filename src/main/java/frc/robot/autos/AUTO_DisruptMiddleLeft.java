package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_DisruptMiddleLeft implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("DisruptMiddle", false, robot.drive),
        // intake and sweep half the middle
        new ParallelCommandGroup(new CMD_Intake(robot.intake), followPath("DisruptMiddle", false)),
        // turn off intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        // shoot for 2 seconds and then sweep middle again
        new CMD_Shoot(
                robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter)
            .withTimeout(3),
        new ParallelCommandGroup(new CMD_Intake(robot.intake), followPath("SweepHub", false)),
        // // turn of intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        // // shoot until auto ends
        new CMD_Shoot(
            robot.drive, robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter));
  }
}
