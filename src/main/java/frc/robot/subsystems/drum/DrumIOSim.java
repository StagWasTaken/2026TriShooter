package frc.robot.subsystems.drum;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class DrumIOSim implements DrumIO {
  private final Debouncer shooterDebouncer = new Debouncer(0.05);

  // Single flywheel sim — drum is one mechanically linked system
  private final FlywheelSim drumSim;

  private final PIDController pidController =
      new PIDController(DrumConstants.kPSim, 0.0, DrumConstants.kDSim);

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(DrumConstants.kSSim, DrumConstants.kVSim);

  private double reference = 0;
  private double hoodRotations = 0;

  // Simple ball sim — just tracks whether a ball is in flight
  private boolean ballActive = false;
  private double ballPosition = 0;

  public DrumIOSim() {
    drumSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(4), // 4 motors mechanically linked
                DrumConstants.kShooterMOI,
                DrumConstants.kGearRatio),
            DCMotor.getNEO(4));
  }

  public void setHoodRotations(double rotations) {
    this.hoodRotations = rotations;
  }

  @Override
  public void updateInputs(DrumIOInputs inputs) {
    inputs.shooterReference = Units.radiansToDegrees(getReference());
    inputs.readyToShoot = isReady();

    inputs.topLeftCurrent = drumSim.getCurrentDrawAmps();
    inputs.topLeftVoltage = drumSim.getInputVoltage();
    inputs.topLeftVelocity = Units.radiansToDegrees(drumSim.getAngularVelocityRadPerSec());
    inputs.topLeftTemp = 0; // not simulated

    // Followers not simulated — zeroed
    inputs.bottomLeftCurrent = 0;
    inputs.bottomLeftTemp = 0;
    inputs.topRightCurrent = 0;
    inputs.topRightTemp = 0;
    inputs.bottomRightCurrent = 0;
    inputs.bottomRightTemp = 0;
  }

  @Override
  public double getReference() {
    return reference;
  }

  @Override
  public double getVelocity() {
    return drumSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    drumSim.setInputVoltage(voltage);
  }

  @Override
  public void setReference(double velocity) {
    this.reference = velocity;
  }

  @Override
  public boolean isReady() {
    return shooterDebouncer.calculate(
        Math.abs(getVelocity() - reference) <= DrumConstants.kStartOnTargetVel);
  }

  @Override
  public void periodic() {
    drumSim.update(0.02);

    drumSim.setInputVoltage(
        pidController.calculate(getVelocity(), reference) + feedforward.calculate(reference));

    if (ballActive) {
      double wheelSurfaceVel = getVelocity() * DrumConstants.kFlywheelRadiusMeters;
      ballPosition += (wheelSurfaceVel / 2.0) * 0.02;
      if (ballPosition >= DrumConstants.getExitDistMeters(hoodRotations)) {
        ballActive = false;
        ballPosition = 0;
      }
    }
  }
}
