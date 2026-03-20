package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_WheelRadiusCharacterization implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(DriveCommands.wheelRadiusCharacterization(robot.drive));
  }
}
