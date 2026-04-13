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
import frc.robot.commands.drive.JoystickDriveAndAimAtTarget;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.utils.constants.FieldConstants;
import frc.robot.utils.custompids.ChassisHeadingController;
import frc.robot.utils.custompids.MapleJoystickDriveInput;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class CMD_ShootFuelSim extends Command {
  private final Drive drive; // Real subsystem for the Aiming Command
  private final AbstractDriveTrainSimulation driveSim; // Sim object for the Physics
  private final MapleJoystickDriveInput driveSupplier;

  private Command driveCommand;
  private int timer;
  private int shooterIndex = 0;

  private static final Translation2d[] SHOOTER_OFFSETS = {
    new Translation2d(Units.inchesToMeters(7), 0),
    new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(6)),
    new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-6))
  };

  /**
   * @param drive The real Drive subsystem (required for the Aiming Command)
   * @param driveSim The simulation drivetrain (required for physics/pose)
   * @param driveSupplier The joystick input
   */
  public CMD_ShootFuelSim(
      Drive drive, AbstractDriveTrainSimulation driveSim, MapleJoystickDriveInput driveSupplier) {
    this.drive = drive;
    this.driveSim = driveSim;
    this.driveSupplier = driveSupplier;

    // We only add the requirement if we want to "take over" the drive subsystem
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer = 0;

    // Now passing 'drive' (the subsystem) instead of 'driveSim'
    this.driveCommand =
        JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
            driveSupplier,
            drive,
            FieldConstants::getHubPose,
            ShooterConstants.kShooterOptimization,
            0.5,
            false);

    driveCommand.initialize();
  }

  @Override
  public void execute() {
    if (!RobotBase.isSimulation()) return;

    // Run the real aiming logic
    driveCommand.execute();

    // Check alignment via the global Heading Controller
    boolean isAligned = ChassisHeadingController.getInstance().atSetPoint();

    if (isAligned && timer >= 3 && IntakeIOSim.numObjectsInHopper() > 0) {
      // Use the simulation's current pose for physics spawning
      Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();

      ShootingParams params = getShootingParamsWithPrediction(robotPose);
      double launchSpeedMPS = rpmToLaunchVelocity(params.shooterReference());

      IntakeIOSim.obtainFuelFromHopper();

      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new RebuiltFuelOnFly(
                  robotPose.getTranslation(),
                  SHOOTER_OFFSETS[shooterIndex],
                  driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  robotPose.getRotation(),
                  Inches.of(21),
                  MetersPerSecond.of(launchSpeedMPS),
                  Radians.of(params.hoodReference())));

      shooterIndex = (shooterIndex + 1) % 3;
      timer = 0;
    } else {
      timer++;
    }
  }

  private ShootingParams getShootingParamsWithPrediction(Pose2d robotPose) {
    double currentDist = FieldConstants.getHubPose().getDistance(robotPose.getTranslation());
    ShootingParams initial = ShooterConstants.getSimShootingParams(currentDist);

    ChassisSpeeds speeds = driveSim.getDriveTrainSimulatedChassisSpeedsRobotRelative();
    var robotAngle = robotPose.getRotation();

    double vxField =
        speeds.vxMetersPerSecond * robotAngle.getCos()
            - speeds.vyMetersPerSecond * robotAngle.getSin();
    double vyField =
        speeds.vxMetersPerSecond * robotAngle.getSin()
            + speeds.vyMetersPerSecond * robotAngle.getCos();

    Translation2d predictedPos =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(vxField * initial.tofSeconds(), vyField * initial.tofSeconds()));

    return ShooterConstants.getSimShootingParams(
        FieldConstants.getHubPose().getDistance(predictedPos));
  }

  private double rpmToLaunchVelocity(double rpm) {
    return (rpm * 2 * Math.PI * Units.inchesToMeters(2) / 60.0) * 0.435;
  }

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) driveCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return IntakeIOSim.numObjectsInHopper() <= 0;
  }
}
