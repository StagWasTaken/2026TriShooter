package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Robot;
import frc.robot.Robot.RobotName;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.SparkUtil;

public class KickerIOSpark implements KickerIO {
  private final SparkMax kickerMotorLeader;
  private final SparkMax kickerMotorFollower;
  private final RelativeEncoder kickerEncoder;
  private final SparkClosedLoopController kickerController;

  private double kickerReference;
  private ControlType kickerType;

  private LoggedTunableNumber kS, kV, kP, kD;

  public KickerIOSpark() {
    kickerMotorLeader = new SparkMax(KickerConstants.kKickerLeadCanId, MotorType.kBrushless);

    kickerMotorLeader.configure(
        KickerConfig.kickerLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    if (Robot.CURRENT_ROBOT != RobotName.DRUM_BOT) {
      kickerMotorFollower =
          new SparkMax(KickerConstants.kKickerBottomRightanId, MotorType.kBrushless);
      kickerMotorFollower.configure(
          KickerConfig.kickerBottomRightonfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    } else {
      kickerMotorFollower = null;
    }

    kickerController = kickerMotorLeader.getClosedLoopController();
    kickerEncoder = kickerMotorLeader.getEncoder();

    kickerReference = 0;
    kickerType = ControlType.kVoltage;

    if (Robot.tuningMode) {
      kS = new LoggedTunableNumber("Kicker/kS", KickerConstants.kS);
      kV = new LoggedTunableNumber("Kicker/kV", KickerConstants.kV);
      kP = new LoggedTunableNumber("Kicker/kP", KickerConstants.kP);
      kD = new LoggedTunableNumber("Kicker/kD", KickerConstants.kD);
    }
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerReference = getReference();
    inputs.kickerCurrent = getCurrent();
    inputs.kickerVoltage = getVoltage();
    inputs.kickerVelocity = getVelocity();
    inputs.atVelocity = atVelocity();
    inputs.kickerTemp = Fahrenheit.convertFrom(kickerMotorLeader.getMotorTemperature(), Celsius);

    if (kickerMotorFollower != null) {
      inputs.kickerBottomRighturrent = kickerMotorFollower.getOutputCurrent();
      inputs.kickerFollowerTemp =
          Fahrenheit.convertFrom(kickerMotorFollower.getMotorTemperature(), Celsius);
    } else {
      inputs.kickerBottomRighturrent = 0;
      inputs.kickerFollowerTemp = 0;
    }
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
    double ff = 0;

    if (Robot.tuningMode) {
      ff = kS.get() + (kV.get() * getReference());

      if (kP.hasChanged(kP.hashCode()) || kD.hasChanged(kD.hashCode())) {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.closedLoop.pid(kP.get(), 0.0, kD.get());
        setVoltage(0);
        SparkUtil.tryUntilOk(
            kickerMotorLeader,
            5,
            () ->
                kickerMotorLeader.configure(
                    newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
      }
    } else {
      ff = KickerConstants.kS + (KickerConstants.kV * kickerReference);
    }

    if (kickerType == ControlType.kVelocity) {
      kickerController.setSetpoint(
          kickerReference, kickerType, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    } else {
      kickerController.setSetpoint(kickerReference, kickerType, ClosedLoopSlot.kSlot0);
    }
  }
}
