package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.CMD_Intake;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_3MeterTest implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot, boolean mirrored)
      throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("3MeterTest", mirrored, robot.drive),
        new CMD_Intake(robot.intake),
        followPath("3MeterTest", mirrored));
  }
}
