package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.CMD_Extend;
import frc.robot.commands.CMD_Intake;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_2SweepLeft implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("FirstSweep2Sweep", false, robot.drive),
        new ParallelCommandGroup(
            new CMD_Intake(robot.intake), followPath("FirstSweep2Sweep", false)),
        followPath("ShootFirstCycle2Sweep", false),
        new CMD_Extend(robot.intake),
        robot.shootClose().withTimeout(3),
        new ParallelCommandGroup(
            new CMD_Intake(robot.intake), followPath("SecondSweep2Sweep", false)),
        followPath("ShootSecondCycle2Sweep", false),
        new CMD_Extend(robot.intake),
        robot.shootClose());
  }
}
