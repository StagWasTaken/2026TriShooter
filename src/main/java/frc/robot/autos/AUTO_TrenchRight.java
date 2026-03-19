package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.utils.constants.FieldConstants;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_TrenchRight implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("SweepHalfMiddle", false, robot.drive),
        // intake and sweep half the middle
        new ParallelCommandGroup(
            new CMD_Intake(robot.intake), followPath("SweepHalfMiddle", false)),
        // turn off intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        followPath("ShootTrench", false),
        new WaitCommand(0.25),
        robot.drive.alignToTarget(() -> FieldConstants.getHubPose()).withTimeout(1),
        // shoot for 2 seconds and then sweep middle again
        robot.shootClose().withTimeout(3),
        new ParallelCommandGroup(new CMD_Intake(robot.intake), followPath("SweepAgain", false)),
        // turn of intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        followPath("ShootTrenchAgain", false),
        new WaitCommand(0.25),
        // robot.drive.alignToTarget(() -> FieldConstants.getHubPose()).withTimeout(1),
        // shoot until auto ends
        robot.shootClose());
  }
}
