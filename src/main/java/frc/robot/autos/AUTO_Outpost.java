package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_Outpost implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("IntakeOutpost", false, robot.drive),
        // run to outpost with hopper out
        new CMD_Extend(robot.intake),
        followPath("IntakeOutpost", false),
        // wait for HP to unload outpost into hopper
        new WaitCommand(1),
        // shoot until auto ends
        followPath("ShootFromOutpost", false),
        robot.shootClose());
  }
}
