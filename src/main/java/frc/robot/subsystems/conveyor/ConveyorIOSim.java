package frc.robot.subsystems.conveyor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ConveyorIOSim implements ConveyorIO {

  private final DCMotorSim conveyorSim;
  private final PIDController conveyorPIDController =
      new PIDController(ConveyorConstants.kPSim, ConveyorConstants.kISim, ConveyorConstants.kDSim);
  private final SimpleMotorFeedforward conveyorFeedforward =
      new SimpleMotorFeedforward(
          ConveyorConstants.kSSim, ConveyorConstants.kVSim, ConveyorConstants.kASim);
  private double reference = 0;
  public static double objectsInHopper = 0;

  public ConveyorIOSim() {
    conveyorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), .178, ConveyorConstants.kGearRatio),
            DCMotor.getNeo550(1));
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.conveyorCurrent = getCurrent();
    inputs.conveyorVoltage = getVoltage();
    inputs.conveyorReference = getReference();
    inputs.conveyorVelocity = Units.radiansToDegrees(getVelocity());
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
    conveyorSim.setInputVoltage(voltage);
  }

  @Override
  public double getVoltage() {
    return conveyorSim.getInputVoltage();
  }

  @Override
  public double getCurrent() {
    return conveyorSim.getCurrentDrawAmps();
  }

  @Override
  public void periodic() {
    conveyorSim.setInput(
        conveyorPIDController.calculate(conveyorSim.getAngularVelocityRadPerSec(), reference)
            + conveyorFeedforward.calculateWithVelocities(getVelocity(), reference));

    conveyorSim.update(0.02);
  }
}
