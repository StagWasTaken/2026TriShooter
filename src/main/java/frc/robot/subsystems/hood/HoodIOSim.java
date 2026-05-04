package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim hoodSim;

  private final PIDController hoodPIDController =
      new PIDController(HoodConstants.kPSim, 0.0, HoodConstants.kDSim);

  private double reference = HoodConstants.kMinHoodAngle;

  public HoodIOSim() {
    hoodSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNeo550(1), HoodConstants.kHoodMOI, HoodConstants.kGearRatio),
            DCMotor.getNeo550(1),
            HoodConstants.kGearRatio,
            HoodConstants.kHoodRadius,
            HoodConstants.kMinHoodAngle,
            HoodConstants.kMaxHoodAngle,
            true,
            HoodConstants.kStartHoodAngle);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs, PowerDistribution pdh) {
    inputs.hoodCurrent = getCurrent();
    inputs.hoodVoltage = getVoltage();
    inputs.hoodReference = Units.radiansToDegrees(getReference());
    inputs.hoodVelocity = Units.radiansToDegrees(getVelocity());
    inputs.hoodPos = Units.radiansToDegrees(hoodSim.getAngleRads());
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
  public double getPosition() {
    return hoodSim.getAngleRads();
  }

  @Override
  public void setVoltage(double voltage) {
    hoodSim.setInputVoltage(voltage);
  }

  @Override
  public double getCurrent() {
    return hoodSim.getCurrentDrawAmps();
  }

  @Override
  public double getVelocity() {
    return hoodSim.getVelocityRadPerSec();
  }

  @Override
  public boolean atReference() {
    return Math.abs(getReference() - getPosition()) < HoodConstants.kTolerance;
  }

  @Override
  public void periodic() {
    hoodSim.update(0.02);

    hoodSim.setInputVoltage(hoodPIDController.calculate(hoodSim.getAngleRads(), reference));
  }
}
