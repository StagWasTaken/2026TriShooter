package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.utils.constants.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class CMD_ShootFuelSim extends Command {
  private final AbstractDriveTrainSimulation driveSim;
  private int timer;

  private static final Translation2d CENTER_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), 0);
  private static final Translation2d LEFT_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(6));
  private static final Translation2d RIGHT_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-6));
  private static final Translation2d[] SHOOTER_OFFSETS = {
    CENTER_SHOOTER_OFFSET, LEFT_SHOOTER_OFFSET, RIGHT_SHOOTER_OFFSET
  };

  public CMD_ShootFuelSim(AbstractDriveTrainSimulation driveSim) {
    this.driveSim = driveSim;
  }

  @Override
  public void initialize() {
    timer = 0;
    if (!RobotBase.isSimulation()) return;
  }

  private int shooterIndex = 0;

  @Override
  public void execute() {
    if (!RobotBase.isSimulation()) return;

    if (timer >= 3 && IntakeIOSim.numObjectsInHopper() > 0) {
      Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
      Translation2d shooterOffset = SHOOTER_OFFSETS[shooterIndex];

      ShootingParams params =
          calculateLeadingParams(robotPose, shooterOffset, FieldConstants.getHubPose());

      IntakeIOSim.obtainFuelFromHopper();

      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new RebuiltFuelOnFly(
                  robotPose.getTranslation(),
                  shooterOffset,
                  driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  robotPose.getRotation(),
                  Inches.of(21),
                  MetersPerSecond.of(params.shooterReference()),
                  Radians.of(params.hoodReference())));

      shooterIndex = (shooterIndex + 1) % 3;
      timer = 0;
    } else {
      timer++;
    }
  }

  @Override
  public boolean isFinished() {
    return IntakeIOSim.numObjectsInHopper() <= 0;
  }

  private ShootingParams calculateLeadingParams(
      Pose2d robotPose, Translation2d shooterOffset, Translation2d hubPosition) {
    Translation2d shooterOffsetField = shooterOffset.rotateBy(robotPose.getRotation());
    Translation2d shooterPos = robotPose.getTranslation().plus(shooterOffsetField);

    ChassisSpeeds speeds = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    Translation2d relativeVel = robotVel.unaryMinus();

    double distance = shooterPos.getDistance(hubPosition);
    ShootingParams params = ShooterConstants.getShootingParams(distance);

    Translation2d targetPos = hubPosition;

    for (int i = 0; i < 10; i++) {
      double predictedDistance = shooterPos.getDistance(targetPos);
      params = ShooterConstants.getSimShootingParams(predictedDistance);

      double projSpeed = Math.cos(params.hoodReference()) * params.shooterReference();

      Translation2d toTarget = targetPos.minus(shooterPos);
      double t = toTarget.getNorm() / projSpeed;
      targetPos = hubPosition.plus(relativeVel.times(t));

      Logger.recordOutput("Lead/currentDistance", distance);
      Logger.recordOutput("Lead/predictedDistance", predictedDistance);
      Logger.recordOutput("Lead/flightTime", params.tofSeconds());
      Logger.recordOutput("Lead/robotVelX", robotVel.getX());
      Logger.recordOutput("Lead/robotVelY", robotVel.getY());
    }

    return params;
  }
}
