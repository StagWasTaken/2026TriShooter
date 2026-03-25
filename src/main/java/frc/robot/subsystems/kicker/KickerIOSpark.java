package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;

public class KickerIOSpark implements KickerIO {
  private final SparkMax kickerMotorLeader;
  private final SparkMax kickerMotorFollower;
  private final RelativeEncoder kickerEncoder;
  private final SparkClosedLoopController kickerController;

  private double kickerReference;
  private ControlType kickerType;

  public KickerIOSpark() {
    // initialize motor
    kickerMotorLeader = new SparkMax(KickerConstants.kKickerLeadCanId, MotorType.kBrushless);
    kickerMotorFollower = new SparkMax(KickerConstants.kKickerFollowerCanId, MotorType.kBrushless);

    // initialize PID controller
    kickerController = kickerMotorLeader.getClosedLoopController();

    // initalize encoder
    kickerEncoder = kickerMotorLeader.getEncoder();

    // apply config
    kickerMotorLeader.configure(
        KickerConfig.kickerLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    kickerMotorFollower.configure(
        KickerConfig.kickerFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // reset target speed in init
    kickerReference = 0;
    kickerType = ControlType.kVoltage;
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerReference = Units.radiansToDegrees(getReference());
    inputs.kickerCurrent = getCurrent();
    inputs.kickerVoltage = getVoltage();
    inputs.kickerVelocity = Units.radiansToDegrees(getVelocity());
    inputs.atVelocity = atVelocity();
    inputs.kickerTemp = Fahrenheit.convertFrom(kickerMotorLeader.getMotorTemperature(), Celsius);
  }

  @Override
  public double getVelocity() {
    return kickerEncoder.getVelocity();
  }

  @Override
  public double getCurrent() {
    return kickerMotorLeader.getOutputCurrent();
  }

  @Override
  public double getVoltage() {
    return kickerMotorLeader.getBusVoltage() * kickerMotorLeader.getAppliedOutput();
  }

  @Override
  public double getReference() {
    return kickerReference;
  }

  @Override
  public void setVoltage(double voltage) {
    kickerReference = voltage;
    kickerType = ControlType.kVoltage;
  }

  @Override
  public void setReference(double velocity) {
    kickerReference = velocity;
    kickerType = ControlType.kVelocity;
  }

  @Override
  public boolean atVelocity() {
    return Math.abs(getReference() - getVelocity()) <= KickerConstants.kTolerance;
  }

  @Override
  public void periodic() {
    kickerController.setSetpoint(kickerReference, kickerType, ClosedLoopSlot.kSlot0);
  }
}
