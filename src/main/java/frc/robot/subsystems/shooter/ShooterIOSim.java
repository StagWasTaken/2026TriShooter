package frc.robot.subsystems.shooter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final Debouncer shooterDebouncer = new Debouncer(.05);

  private final FlywheelSim middleShooterSim;
  private final FlywheelSim leftShooterSim;
  private final FlywheelSim rightShooterSim;

  private final BallSim middleBall = new BallSim();
  private final BallSim leftBall = new BallSim();
  private final BallSim rightBall = new BallSim();

  private PIDController leftShooterPIDController =
      new PIDController(ShooterConstants.kPSim, 0.0, ShooterConstants.kDSim);
  private PIDController middleShooterPIDController =
      new PIDController(ShooterConstants.kPSim, 0.0, ShooterConstants.kDSim);
  private PIDController rightShooterPIDController =
      new PIDController(ShooterConstants.kPSim, 0.0, ShooterConstants.kDSim);

  private SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.kSSim, ShooterConstants.kVSim);

  private double reference = 0;
  private double hoodRotations = 0;
  public static double objectsInHopper = 0;

  public ShooterIOSim() {
    middleShooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), ShooterConstants.kShooterMOI, ShooterConstants.kGearRatio),
            DCMotor.getNEO(1));

    leftShooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), ShooterConstants.kShooterMOI, ShooterConstants.kGearRatio),
            DCMotor.getNEO(1));

    rightShooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), ShooterConstants.kShooterMOI, ShooterConstants.kGearRatio),
            DCMotor.getNEO(1));
  }

  /** Helper class to manage ball travel and torque loading */
  private class BallSim {
    double position = 0;
    boolean active = false;

    void update(FlywheelSim sim, double dt, double exitDist) {
      if (!active) return;

      // Ball center moves at half surface speed: (omega * r) / 2
      double wheelSurfaceVel =
          sim.getAngularVelocityRadPerSec() * ShooterConstants.kFlywheelRadiusMeters;
      position += (wheelSurfaceVel / 2.0) * dt;

      // Apply load torque: FrictionForce * Radius
      // 130N * 0.75 * 0.0508m = ~4.95Nm
      double loadTorque =
          ShooterConstants.kNormalForceNewtons
              * ShooterConstants.kWheelCOF
              * ShooterConstants.kFlywheelRadiusMeters;

      // Apply current load to the sim
      sim.setInputVoltage(sim.getInputVoltage()); // Maintain set voltage

      // FlywheelSim doesn't have an 'applyTorque' method, but we can manually
      // decelerate the state based on T = J * alpha -> alpha = T / J
      double deceleration = (loadTorque / ShooterConstants.kShooterMOI) * dt;
      sim.setState(VecBuilder.fill(sim.getAngularVelocityRadPerSec() - deceleration));

      if (position >= exitDist) {
        active = false;
        position = 0;
      }
    }
  }

  public void spawnMiddleBall() {
    middleBall.active = true;
    middleBall.position = 0;
  }

  public void spawnLeftBall() {
    leftBall.active = true;
    leftBall.position = 0;
  }

  public void spawnRightBall() {
    rightBall.active = true;
    rightBall.position = 0;
  }

  public void setHoodRotations(double rotations) {
    this.hoodRotations = rotations;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterReference = Units.radiansToDegrees(getReference());
    inputs.readyToShoot = isReady();

    inputs.middleShooterCurrent = getMiddleCurrent();
    inputs.middleShooterVoltage = getMiddleVoltage();
    inputs.middleShooterVelocity = Units.radiansToDegrees(getMiddleVelocity());

    inputs.leftShooterCurrent = getLeftCurrent();
    inputs.leftShooterVoltage = getLeftVoltage();
    inputs.leftShooterVelocity = Units.radiansToDegrees(getLeftVelocity());

    inputs.rightShooterCurrent = getRightCurrent();
    inputs.rightShooterVoltage = getRightVoltage();
    inputs.rightShooterVelocity = Units.radiansToDegrees(getRightVelocity());
  }

  @Override
  public void setReference(double reference) {
    this.reference = reference;
  }

  @Override
  public double getReference() {
    return reference;
  }

  @Override
  public void setVoltage(double voltage) {
    middleShooterSim.setInputVoltage(voltage);
    leftShooterSim.setInputVoltage(voltage);
    rightShooterSim.setInputVoltage(voltage);
  }

  @Override
  public double getMiddleVoltage() {
    return middleShooterSim.getInputVoltage();
  }

  @Override
  public double getMiddleCurrent() {
    return middleShooterSim.getCurrentDrawAmps();
  }

  @Override
  public double getMiddleVelocity() {
    return middleShooterSim.getAngularVelocityRadPerSec();
  }

  @Override
  public double getLeftVoltage() {
    return leftShooterSim.getInputVoltage();
  }

  @Override
  public double getLeftCurrent() {
    return leftShooterSim.getCurrentDrawAmps();
  }

  @Override
  public double getLeftVelocity() {
    return leftShooterSim.getAngularVelocityRadPerSec();
  }

  @Override
  public double getRightVoltage() {
    return rightShooterSim.getInputVoltage();
  }

  @Override
  public double getRightCurrent() {
    return rightShooterSim.getCurrentDrawAmps();
  }

  @Override
  public double getRightVelocity() {
    return rightShooterSim.getAngularVelocityRadPerSec();
  }

  @Override
  public boolean isReady() {
    boolean leftReadyToShoot =
        Math.abs(getLeftVelocity() - getReference()) <= ShooterConstants.kStartOnTargetVel;
    boolean middleReadyToShoot =
        Math.abs(getMiddleVelocity() - getReference()) <= ShooterConstants.kStartOnTargetVel;
    boolean rightReadyToShoot =
        Math.abs(getRightVelocity() - getReference()) <= ShooterConstants.kStartOnTargetVel;

    return shooterDebouncer.calculate(leftReadyToShoot && middleReadyToShoot && rightReadyToShoot);
  }

  @Override
  public void periodic() {
    double exitDist = ShooterConstants.getExitDistMeters(hoodRotations);

    leftBall.update(leftShooterSim, 0.02, exitDist);
    middleBall.update(middleShooterSim, 0.02, exitDist);
    rightBall.update(rightShooterSim, 0.02, exitDist);

    leftShooterSim.update(.02);
    middleShooterSim.update(0.02);
    rightShooterSim.update(.02);

    leftShooterSim.setInputVoltage(
        leftShooterPIDController.calculate(getLeftVelocity(), reference)
            + shooterFeedforward.calculate(reference));

    middleShooterSim.setInputVoltage(
        middleShooterPIDController.calculate(getMiddleVelocity(), reference)
            + shooterFeedforward.calculate(reference));

    rightShooterSim.setInputVoltage(
        rightShooterPIDController.calculate(getRightVelocity(), reference)
            + shooterFeedforward.calculate(reference));
  }
}
