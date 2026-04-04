package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import frc.robot.Robot;
import frc.robot.Robot.RobotName;

public class HoodIOSpark implements HoodIO {
  private final SparkMax hoodMotor;
  // Only one of these will be non-null depending on robot
  private final AbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController hoodController;

  private double hoodReference;
  private ControlType hoodType;

  public HoodIOSpark() {
    hoodMotor = new SparkMax(HoodConstants.kHoodCanId, MotorType.kBrushless);
    hoodController = hoodMotor.getClosedLoopController();

    if (Robot.CURRENT_ROBOT == RobotName.DRUM_BOT) {
      relativeEncoder = hoodMotor.getEncoder();
      absoluteEncoder = null;
      hoodMotor.configure(
          HoodConfig.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else {
      absoluteEncoder = hoodMotor.getAbsoluteEncoder();
      relativeEncoder = null;
      hoodMotor.configure(
          HoodConfig.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    hoodReference = HoodConstants.kMinPos;
    hoodType = ControlType.kPosition;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodReference = getReference();
    inputs.hoodCurrent = getCurrent();
    inputs.hoodVoltage = getVoltage();
    inputs.hoodVelocity = getVelocity();
    inputs.hoodPos = getPosition();
    inputs.hoodTemp = Fahrenheit.convertFrom(hoodMotor.getMotorTemperature(), Celsius);
    inputs.atReference = atReference();
  }

  @Override
  public double getVelocity() {
    return relativeEncoder != null ? relativeEncoder.getVelocity() : absoluteEncoder.getVelocity();
  }

  @Override
  public double getPosition() {
    return relativeEncoder != null ? relativeEncoder.getPosition() : absoluteEncoder.getPosition();
  }

  @Override
  public double getCurrent() {
    return hoodMotor.getOutputCurrent();
  }

  @Override
  public double getVoltage() {
    return hoodMotor.getBusVoltage() * hoodMotor.getAppliedOutput();
  }

  @Override
  public double getReference() {
    return hoodReference;
  }

  @Override
  public void setVoltage(double voltage) {
    hoodReference = voltage;
    hoodType = ControlType.kVoltage;
  }

  @Override
  public void setReference(double reference) {
    hoodReference = MathUtil.clamp(reference, HoodConstants.kMinPos, HoodConstants.kMaxPos);
    hoodType = ControlType.kPosition;
  }

  @Override
  public boolean atReference() {
    return Math.abs(getReference() - getPosition()) < HoodConstants.kTolerance;
  }

  @Override
  public void resetEncoder() {
    if (relativeEncoder != null) {
      relativeEncoder.setPosition(0);
    }
  }

  @Override
  public void periodic() {
    hoodController.setSetpoint(hoodReference, hoodType, ClosedLoopSlot.kSlot0);
  }
}
