package frc.robot.subsystems.drum;

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
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Robot;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.SparkUtil;

public class DrumIOSpark implements DrumIO {
  private final SparkMax topLeftLeader;
  private final SparkMax bottomLeftFollower, topRightFollower, bottomRightFollower;

  private final RelativeEncoder topLeftLeaderEncoder;
  private final RelativeEncoder bottomLeftFollowerEncoder;
  private final RelativeEncoder topRightFollowerEncoder;
  private final RelativeEncoder bottomRightFollowerEncoder;

  private final SparkClosedLoopController topLeftLeaderController;
  private final SparkClosedLoopController bottomLeftFollowerController;
  private final SparkClosedLoopController topRightFollowerController;
  private final SparkClosedLoopController bottomRightFollowerController;

  private double shooterReference;
  private ControlType shooterType;

  private LoggedTunableNumber kS, kV, kP, kD;
  private final Debouncer drumDebouncer = new Debouncer(0.05);
  public boolean shooting;

  public DrumIOSpark() {
    topLeftLeader = new SparkMax(DrumConstants.kTopLeftLeaderCanId, MotorType.kBrushless);
    bottomLeftFollower = new SparkMax(DrumConstants.kBottomLeftFollowerCanId, MotorType.kBrushless);
    topRightFollower = new SparkMax(DrumConstants.kTopRightFollowerCanId, MotorType.kBrushless);
    bottomRightFollower =
        new SparkMax(DrumConstants.kBottomRightFollowerCanId, MotorType.kBrushless);

    topLeftLeaderEncoder = topLeftLeader.getEncoder();
    bottomLeftFollowerEncoder = bottomLeftFollower.getEncoder();
    topRightFollowerEncoder = topRightFollower.getEncoder();
    bottomRightFollowerEncoder = bottomRightFollower.getEncoder();

    topLeftLeaderController = topLeftLeader.getClosedLoopController();
    bottomLeftFollowerController = bottomLeftFollower.getClosedLoopController();
    topRightFollowerController = topRightFollower.getClosedLoopController();
    bottomRightFollowerController = bottomRightFollower.getClosedLoopController();

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

    if (Robot.tuningMode) {
      kS = new LoggedTunableNumber("Drum/kS", DrumConstants.kS);
      kV = new LoggedTunableNumber("Drum/kV", DrumConstants.kV);
      kP = new LoggedTunableNumber("Drum/kP", DrumConstants.kP);
      kD = new LoggedTunableNumber("Drum/kD", DrumConstants.kD);
    }
  }

  @Override
  public void updateInputs(DrumIOInputs inputs) {
    inputs.shooterReference = getReference() / ((Math.PI * 2) / 60);
    inputs.readyToShoot = isReady();

    inputs.leaderCurrent = topLeftLeader.getOutputCurrent();
    inputs.leaderVoltage = topLeftLeader.getBusVoltage() * topLeftLeader.getAppliedOutput();
    inputs.leaderVelocity = topLeftLeaderEncoder.getVelocity() / ((Math.PI * 2) / 60);
    inputs.leaderTemp = Fahrenheit.convertFrom(topLeftLeader.getMotorTemperature(), Celsius);

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

  private void setAllSetpoints(double reference, ControlType type, double ff) {
    topLeftLeaderController.setSetpoint(
        reference, type, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    bottomLeftFollowerController.setSetpoint(
        reference, type, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    topRightFollowerController.setSetpoint(
        reference, type, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    bottomRightFollowerController.setSetpoint(
        reference, type, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
  }

  private void setAllVoltage(double voltage) {
    topLeftLeaderController.setSetpoint(voltage, ControlType.kVoltage);
    bottomLeftFollowerController.setSetpoint(voltage, ControlType.kVoltage);
    topRightFollowerController.setSetpoint(voltage, ControlType.kVoltage);
    bottomRightFollowerController.setSetpoint(voltage, ControlType.kVoltage);
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
        for (SparkMax motor :
            new SparkMax[] {
              topLeftLeader, bottomLeftFollower, topRightFollower, bottomRightFollower
            }) {
          SparkUtil.tryUntilOk(
              motor,
              5,
              () ->
                  motor.configure(
                      newConfig,
                      ResetMode.kNoResetSafeParameters,
                      PersistMode.kNoPersistParameters));
        }
      }
    } else {
      ff = DrumConstants.kS + (DrumConstants.kV * getReference());
    }

    if (shooterReference > 0) {
      setAllSetpoints(shooterReference, shooterType, ff);
    } else {
      setAllVoltage(DrumConstants.kIdleVolts);
    }
  }
}
