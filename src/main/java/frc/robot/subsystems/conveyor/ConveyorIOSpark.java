package frc.robot.subsystems.conveyor;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;

public class ConveyorIOSpark implements ConveyorIO {
  private final SparkMax conveyorMotor;
  private final RelativeEncoder conveyorEncoder;
  private final SparkClosedLoopController conveyorController;

  private double conveyorReference;
  private ControlType conveyorType;

  public ConveyorIOSpark() {
    // initialize motor
    conveyorMotor = new SparkMax(ConveyorConstants.kConveyorCanId, MotorType.kBrushless);

    // initialize PID controller
    conveyorController = conveyorMotor.getClosedLoopController();

    // initalize encoder
    conveyorEncoder = conveyorMotor.getEncoder();

    // apply config
    conveyorMotor.configure(
        ConveyorConfig.conveyorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // reset target speed in init
    conveyorReference = 0;
    conveyorType = ControlType.kVoltage;
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs, PowerDistribution pdh) {
    inputs.conveyorReference = getReference();
    inputs.conveyorCurrent = getCurrent();
    inputs.conveyorSupplyCurrent = pdh.getCurrent(conveyorMotor.getDeviceId());
    inputs.conveyorVoltage = getVoltage();
    inputs.conveyorVelocity = getVelocity();
    inputs.conveyorTemp = Fahrenheit.convertFrom(conveyorMotor.getMotorTemperature(), Celsius);
  }

  @Override
  public double getVelocity() {
    return conveyorEncoder.getVelocity();
  }

  @Override
  public double getCurrent() {
    return conveyorMotor.getOutputCurrent();
  }

  @Override
  public double getVoltage() {
    return conveyorMotor.getBusVoltage() * conveyorMotor.getAppliedOutput();
  }

  @Override
  public double getReference() {
    return conveyorReference;
  }

  @Override
  public void setVoltage(double voltage) {
    conveyorReference = voltage;
    conveyorType = ControlType.kVoltage;
  }

  @Override
  public void setReference(double velocity) {
    conveyorReference = velocity;
    conveyorType = ControlType.kVelocity;
  }

  @Override
  public void periodic() {
    conveyorController.setSetpoint(conveyorReference, conveyorType);
  }
}
