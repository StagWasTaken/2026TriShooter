package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.utils.constants.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class CMD_ShootFuelSimulatedForces extends Command {
  private final AbstractDriveTrainSimulation driveSim;
  private final Hood hood;
  private final Shooter shooter;

  private static final Translation2d CENTER_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), 0);
  private static final Translation2d LEFT_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(6));
  private static final Translation2d RIGHT_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-6));
  private static final Translation2d[] SHOOTER_OFFSETS = {
    CENTER_SHOOTER_OFFSET, LEFT_SHOOTER_OFFSET, RIGHT_SHOOTER_OFFSET
  };

  public CMD_ShootFuelSimulatedForces(
      AbstractDriveTrainSimulation driveSim, Hood hood, Shooter shooter) {
    this.driveSim = driveSim;
    this.hood = hood;
    this.shooter = shooter;
    addRequirements(shooter, hood);
  }

  @Override
  public void initialize() {
    if (!RobotBase.isSimulation()) return;
  }

  private int shooterIndex = 0;

  @Override
  public void execute() {
    Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
    double distToHubMeters = robotPose.getTranslation().getDistance(FieldConstants.getHubPose());
    ShootingParams shootingParams = ShooterConstants.getShootingParams(distToHubMeters);

    shooter.setReference(shootingParams.shooterReference());
    hood.setReference(shootingParams.hoodReference());

    if (IntakeIOSim.numObjectsInHopper() > 0 && shooter.isReady()) {
      Translation2d shooterOffset = SHOOTER_OFFSETS[shooterIndex];

      // Trigger the Physics Simulation "Dip"
      shooter.spawnSimulatedBall(shooterIndex);

      // Calculate actual exit velocity,
      // In a hood shooter, the ball exits at 1/2 the surface velocity of the wheel
      double currentRadPerSec = shooter.getVelocityRadPerSec(shooterIndex);
      double wheelSurfaceSpeed = currentRadPerSec * ShooterConstants.kFlywheelRadiusMeters;
      double exitVelocityMps = wheelSurfaceSpeed / 2;

      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new RebuiltFuelOnFly(
                  robotPose.getTranslation(),
                  shooterOffset,
                  driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  robotPose.getRotation(),
                  Inches.of(21),
                  MetersPerSecond.of(exitVelocityMps),
                  Radians.of(hood.getPosition())));

      IntakeIOSim.obtainFuelFromHopper();
      shooterIndex = (shooterIndex + 1) % 3;
    }
  }

  @Override
  public boolean isFinished() {
    return IntakeIOSim.numObjectsInHopper() <= 0;
  }
}
