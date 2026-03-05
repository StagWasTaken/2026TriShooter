package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim intakeSim;
  private final PIDController intakePIDController =
      new PIDController(IntakeConstants.kPSim, IntakeConstants.kISim, IntakeConstants.kDSim);
  private final SimpleMotorFeedforward intakeFeedforward =
      new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);
  private static IntakeSimulation intakeSimulation;
  private double reference = 0;

  private final SingleJointedArmSim intakeExtenderSim;
  private final PIDController intakeExtenderPIDController =
      new PIDController(ExtenderConstants.kPSim, ExtenderConstants.kISim, ExtenderConstants.kDSim);
  private final ArmFeedforward intakeExtenderFeedforward =
      new ArmFeedforward(
          ExtenderConstants.kSSim,
          ExtenderConstants.kGSim,
          ExtenderConstants.kVSim,
          ExtenderConstants.kASim);

  private final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(Math.PI, Math.PI * 4));

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(ExtenderConstants.kHome, 0);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(ExtenderConstants.kStow, 0);
  private double appliedExtenderVoltage = 0.0;

  private final LoggedMechanism2d intakeMechanism;
  private final LoggedMechanismRoot2d intakeRoot;
  private final LoggedMechanismLigament2d intakeVisualizer =
      new LoggedMechanismLigament2d(
          "intake", Inches.of(1), Degrees.of(-87.5), 50, new Color8Bit(255, 92, 0));

  public IntakeIOSim(AbstractDriveTrainSimulation driveSim) {
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeoVortex(2), .178, IntakeConstants.kGearRatio),
            DCMotor.getNeoVortex(2));

    intakeExtenderSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1), .3, ExtenderConstants.kGearRatio),
            DCMotor.getNEO(1),
            ExtenderConstants.kGearRatio,
            Units.inchesToMeters(12),
            ExtenderConstants.kHome - Math.toRadians(270),
            0,
            true,
            ExtenderConstants.kHome - Math.toRadians(270));

    intakeMechanism = new LoggedMechanism2d(Inches.of(24), Inches.of(10));
    intakeRoot =
        intakeMechanism.getRoot("Intake", Units.inchesToMeters(-16), Units.inchesToMeters(0));
    intakeRoot.append(intakeVisualizer);

    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSim, Inches.of(24), Inches.of(10), IntakeSide.BACK, 48);

    intakeSimulation.startIntake();

    intakeExtenderSim.setState(ExtenderConstants.kHome - Math.toRadians(270), 0);

    setExtenderReference(ExtenderConstants.kExtended);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeCurrent = getCurrent();
    inputs.intakeVoltage = getVoltage();
    inputs.intakeReference = getReference();
    inputs.intakeVelocity = Units.radiansToDegrees(getVelocity());
    inputs.intakePosition = Units.radiansToDegrees(getPosition());

    inputs.extenderCurrent = getExtenderCurrent();
    inputs.extenderVoltage = getExtenderVoltage();
    inputs.extenderReference = Units.radiansToDegrees(getExtenderReference());
    inputs.extenderVelocity = Units.radiansToDegrees(getExtenderVelocity());
    inputs.extenderPosition = Units.radiansToDegrees(getExtenderPosition());
    inputs.extenderProfilePositionSetpoint = Units.radiansToDegrees(setpoint.position);
    inputs.extenderProfileVelocitySetpoint = Units.radiansToDegrees(setpoint.velocity);
    inputs.extenderInPosition = getExtenderInPosition();
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
    intakeSim.setInputVoltage(voltage);
  }

  @Override
  public double getVoltage() {
    return intakeSim.getInputVoltage();
  }

  @Override
  public double getCurrent() {
    return intakeSim.getCurrentDrawAmps();
  }

  @Override
  public double getVelocity() {
    return intakeSim.getAngularVelocityRadPerSec();
  }

  @Override
  public double getPosition() {
    return intakeSim.getAngularPositionRad();
  }

  @Override
  public void setExtenderReference(double reference) {
    goal = new TrapezoidProfile.State(reference, 0);
  }

  @Override
  public double getExtenderReference() {
    return goal.position;
  }

  @Override
  public void setExtenderVoltage(double voltage) {
    intakeExtenderSim.setInputVoltage(voltage);
  }

  @Override
  public double getExtenderVoltage() {
    return appliedExtenderVoltage;
  }

  @Override
  public double getExtenderCurrent() {
    return intakeExtenderSim.getCurrentDrawAmps();
  }

  @Override
  public double getExtenderVelocity() {
    return intakeExtenderSim.getVelocityRadPerSec();
  }

  @Override
  public double getExtenderPosition() {
    return intakeExtenderSim.getAngleRads() + Math.toRadians(270);
  }

  @Override
  public boolean getExtenderInPosition() {
    return Math.abs(getExtenderPosition() - getExtenderReference())
        < ExtenderConstants.kPositionTolerance;
  }

  @Override
  public void periodic() {
    double currentVelocityTarget = setpoint.velocity;
    setpoint = profile.calculate(0.02, setpoint, goal);

    intakeSim.setInputVoltage(
        intakePIDController.calculate(intakeSim.getAngularVelocityRadPerSec(), reference)
            + intakeFeedforward.calculateWithVelocities(getVelocity(), reference));

    appliedExtenderVoltage =
        intakeExtenderPIDController.calculate(getExtenderPosition(), setpoint.position)
            + intakeExtenderFeedforward.calculateWithVelocities(
                getExtenderPosition() - Math.toRadians(270),
                currentVelocityTarget,
                setpoint.velocity);

    setExtenderVoltage(appliedExtenderVoltage);

    intakeSim.update(0.02);
    intakeExtenderSim.update(0.02);

    intakeVisualizer.setAngle(Units.radiansToDegrees(getExtenderPosition()));

    Logger.recordOutput("Intake/FuelInHopper", numObjectsInHopper());
    Logger.recordOutput("IntakeVisualizer", intakeMechanism);
  }

  public static boolean obtainFuelFromHopper() {
    return intakeSimulation.obtainGamePieceFromIntake();
  }

  public static int numObjectsInHopper() {
    return intakeSimulation.getGamePiecesAmount();
  }

  public static void putFuelInHopperSim(int fuel) {
    setFuelInHopper(numObjectsInHopper() + fuel);
  }

  public static void setFuelInHopper(int fuel) {
    intakeSimulation.setGamePiecesCount(fuel);
  }
}
