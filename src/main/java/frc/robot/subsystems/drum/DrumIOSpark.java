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

public class DrumIOSpark implements DrumIO {
  // Leader — all PID/FF runs here
  private final SparkMax topLeftLeader;
  private final RelativeEncoder topLeftLeaderEncoder;
  private final SparkClosedLoopController topLeftLeaderController;

  // Followers — configured via DrumConfig to follow topLeftLeader
  private final SparkMax bottomLeftFollower, topRightFollower, bottomRightFollower;
  private final RelativeEncoder bottomLeftFollowerEncoder;
  private final RelativeEncoder topRightFollowerEncoder;
  private final RelativeEncoder bottomRightFollowerEncoder;

  private double shooterReference;
  private ControlType shooterType;

  private final Debouncer drumDebouncer = new Debouncer(0.05);
  public boolean shooting;

  public DrumIOSpark() {
    topLeftLeader = new SparkMax(DrumConstants.kTopLeftLeaderCanId, MotorType.kBrushless);
    bottomLeftFollower = new SparkMax(DrumConstants.kBottomLeftFollowerCanId, MotorType.kBrushless);
    topRightFollower = new SparkMax(DrumConstants.kTopRightFollowerCanId, MotorType.kBrushless);
    bottomRightFollower =
        new SparkMax(DrumConstants.kBottomRightFollowerCanId, MotorType.kBrushless);

    topLeftLeaderEncoder = topLeftLeader.getEncoder();
    topLeftLeaderController = topLeftLeader.getClosedLoopController();

    bottomLeftFollowerEncoder = bottomLeftFollower.getEncoder();
    topRightFollowerEncoder = topRightFollower.getEncoder();
    bottomRightFollowerEncoder = bottomRightFollower.getEncoder();

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
    shooting = false;
  }

  @Override
  public void updateInputs(DrumIOInputs inputs) {
    inputs.shooterReference = getReference() / ((Math.PI * 2) / 60);
    inputs.readyToShoot = isReady();

    // Leader — used for control and logging
    inputs.leaderCurrent = topLeftLeader.getOutputCurrent();
    inputs.leaderVoltage = topLeftLeader.getBusVoltage() * topLeftLeader.getAppliedOutput();
    inputs.leaderVelocity = topLeftLeaderEncoder.getVelocity() / ((Math.PI * 2) / 60);
    inputs.leaderTemp = Fahrenheit.convertFrom(topLeftLeader.getMotorTemperature(), Celsius);

    // Followers — current and temp only for diagnostics
    inputs.followerACurrent = bottomLeftFollower.getOutputCurrent();
    inputs.followerAVoltage =
        bottomLeftFollower.getBusVoltage() * bottomLeftFollower.getAppliedOutput();
    inputs.followerATemp =
        Fahrenheit.convertFrom(bottomLeftFollower.getMotorTemperature(), Celsius);
    inputs.followerAVel = bottomLeftFollowerEncoder.getVelocity() / ((Math.PI * 2) / 60);

    inputs.followerBCurrent = topRightFollower.getOutputCurrent();
    inputs.followerBVoltage =
        topRightFollower.getBusVoltage() * topRightFollower.getAppliedOutput();
    inputs.followerBTemp = Fahrenheit.convertFrom(topRightFollower.getMotorTemperature(), Celsius);
    inputs.followerBVel = topRightFollowerEncoder.getVelocity() / ((Math.PI * 2) / 60);

    inputs.followerCCurrent = bottomRightFollower.getOutputCurrent();
    inputs.followerCVoltage =
        bottomRightFollower.getBusVoltage() * bottomRightFollower.getAppliedOutput();
    inputs.followerCTemp =
        Fahrenheit.convertFrom(bottomRightFollower.getMotorTemperature(), Celsius);
    inputs.followerCVel = bottomRightFollowerEncoder.getVelocity() / ((Math.PI * 2) / 60);
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
  public void startShooting() {
    shooting = true;
  }

  @Override
  public void stopShooting() {
    shooting = false;
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

    // if (shooting) {
    //     ff += 0.25;
    // }

    // Followers mirror the leader automatically — only need to command the leader
    if (shooterReference > 0) {
      topLeftLeaderController.setSetpoint(shooterReference, shooterType, ClosedLoopSlot.kSlot0, ff);
    } else {
      topLeftLeaderController.setSetpoint(DrumConstants.kIdleVolts, ControlType.kVoltage);
    }
  }
}
