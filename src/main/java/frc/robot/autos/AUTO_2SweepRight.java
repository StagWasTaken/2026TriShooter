package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.CMD_Extend;
import frc.robot.commands.CMD_Intake;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_2SweepRight implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("FirstSweep2Sweep", true, robot.drive),
        new CMD_Intake(robot.intake),
        followPath("FirstSweep2Sweep", true),
        new CMD_Extend(robot.intake),
        followPath("ShootFirstCycle2Sweep", true),
        robot.shootClose().withTimeout(3),
        new CMD_Intake(robot.intake),
        followPath("SecondSweep2Sweep", true),
        new CMD_Extend(robot.intake),
        followPath("ShootSecondCycle2Sweep", true),
        robot.shootClose());
  }
}
