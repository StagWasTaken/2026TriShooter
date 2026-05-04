package frc.robot.subsystems.kicker;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class KickerIOSim implements KickerIO {

  private final DCMotorSim kickerSim;
  private final PIDController kickerPIDController =
      new PIDController(KickerConstants.kPSim, KickerConstants.kISim, KickerConstants.kDSim);
  private final SimpleMotorFeedforward kickerFeedforward =
      new SimpleMotorFeedforward(KickerConstants.kS, KickerConstants.kV, KickerConstants.kA);
  private double reference = 0;
  public static double objectsInHopper = 0;

  public KickerIOSim() {
    kickerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), .178, KickerConstants.kGearRatio),
            DCMotor.getNeo550(2));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs, PowerDistribution pdh) {
    inputs.kickerCurrent = getCurrent();
    inputs.kickerVoltage = getVoltage();
    inputs.kickerReference = getReference();
    inputs.kickerVelocity = Units.radiansToDegrees(getVelocity());
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
    kickerSim.setInputVoltage(voltage);
  }

  @Override
  public double getVoltage() {
    return kickerSim.getInputVoltage();
  }

  @Override
  public double getCurrent() {
    return kickerSim.getCurrentDrawAmps();
  }

  @Override
  public void periodic() {
    kickerSim.setInput(
        kickerPIDController.calculate(kickerSim.getAngularVelocityRadPerSec(), reference)
            + kickerFeedforward.calculateWithVelocities(getVelocity(), reference));

    kickerSim.update(0.02);
  }
}
