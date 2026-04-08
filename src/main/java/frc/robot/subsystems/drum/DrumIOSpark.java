package frc.robot.subsystems.drum;

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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;

public class DrumIOSpark implements DrumIO {
  // Leader — all PID/FF runs here
  private final SparkMax topLeftLeader;
  private final RelativeEncoder topLeftLeaderEncoder;
  private final SparkClosedLoopController topLeftLeaderController;

  // Followers — configured via DrumConfig to follow topLeftLeader
  private final SparkMax bottomLeftFollower, topRightFollower, bottomRightFollower;

  private double shooterReference;
  private ControlType shooterType;

  private final Debouncer drumDebouncer = new Debouncer(0.05);

  public DrumIOSpark() {
    topLeftLeader = new SparkMax(DrumConstants.kTopLeftLeaderCanId, MotorType.kBrushless);
    bottomLeftFollower = new SparkMax(DrumConstants.kBottomLeftFollowerCanId, MotorType.kBrushless);
    topRightFollower = new SparkMax(DrumConstants.kTopRightFollowerCanId, MotorType.kBrushless);
    bottomRightFollower =
        new SparkMax(DrumConstants.kBottomRightFollowerCanId, MotorType.kBrushless);

    topLeftLeaderEncoder = topLeftLeader.getEncoder();
    topLeftLeaderController = topLeftLeader.getClosedLoopController();

    topLeftLeader.configure(
        DrumConfig.topLeftLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    bottomLeftFollower.configure(
        DrumConfig.bottomLeftFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    topRightFollower.configure(
        DrumConfig.topRightFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    bottomRightFollower.configure(
        DrumConfig.bottomRightFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterType = ControlType.kVelocity;
  }

  @Override
  public void updateInputs(DrumIOInputs inputs) {
    inputs.shooterReference = Units.radiansToDegrees(getReference());
    inputs.readyToShoot = isReady();

    // Leader — used for control and logging
    inputs.leaderCurrent = topLeftLeader.getOutputCurrent();
    inputs.leaderVoltage = topLeftLeader.getBusVoltage() * topLeftLeader.getAppliedOutput();
    inputs.leaderVelocity = Units.radiansToDegrees(topLeftLeaderEncoder.getVelocity());
    inputs.leaderTemp = Fahrenheit.convertFrom(topLeftLeader.getMotorTemperature(), Celsius);

    // Followers — current and temp only for diagnostics
    inputs.followerACurrent = bottomLeftFollower.getOutputCurrent();
    inputs.followerATemp =
        Fahrenheit.convertFrom(bottomLeftFollower.getMotorTemperature(), Celsius);

    inputs.followerBCurrent = topRightFollower.getOutputCurrent();
    inputs.followerBTemp = Fahrenheit.convertFrom(topRightFollower.getMotorTemperature(), Celsius);

    inputs.followerCCurrent = bottomRightFollower.getOutputCurrent();
    inputs.followerCTemp =
        Fahrenheit.convertFrom(bottomRightFollower.getMotorTemperature(), Celsius);
  }

  @Override
  public double getReference() {
    return shooterReference;
  }

  @Override
  public double getVelocity() {
    return topLeftLeaderEncoder.getVelocity();
  }

  @Override
  public void setReference(double velocity) {
    shooterReference = velocity;
    shooterType = ControlType.kVelocity;
  }

  @Override
  public void setVoltage(double voltage) {
    shooterReference = voltage;
    shooterType = ControlType.kVoltage;
  }

  @Override
  public boolean isReady() {
    return drumDebouncer.calculate(
        Math.abs(topLeftLeaderEncoder.getVelocity() - getReference())
            < DrumConstants.kStartOnTargetVel);
  }

  @Override
  public void periodic() {
    double ff = 0;

    // tuning

    // double referenceRad = Units.degreesToRadians(reference.get());
    // shooterReference = referenceRad;

    // if (shooterType == ControlType.kVelocity && referenceRad != 0) {
    //   ff = kS.get() + (kV.get() * referenceRad);
    // }

    // double p = kP.get();
    // if(lastP != p){
    //     setVoltage(0);
    //     SparkMaxConfig newConfig = new SparkMaxConfig();
    //     newConfig.apply(DrumConfig.topLeftLeaderConfig);
    //     newConfig.closedLoop.p(p);
    //     topLeftLeader.configureAsync(newConfig, ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    // }
    // lastP = p;

    // topLeftLeaderController.setSetpoint(referenceRad, shooterType, ClosedLoopSlot.kSlot0, ff);

    // real

    if (shooterType == ControlType.kVelocity) {
      ff = DrumConstants.kS + (DrumConstants.kV * getReference());
    }

    // Followers mirror the leader automatically — only need to command the leader
    if (shooterReference > 0) {
      topLeftLeaderController.setSetpoint(shooterReference, shooterType, ClosedLoopSlot.kSlot0, ff);
    } else {
      topLeftLeaderController.setSetpoint(DrumConstants.kIdleVolts, ControlType.kVoltage);
    }
  }
}
