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
  private final SparkMax leaderMotor;
  private final RelativeEncoder leaderEncoder;
  private final SparkClosedLoopController leaderController;

  // Followers — configured via DrumConfig to follow leaderMotor
  private final SparkMax followerMotorA, followerMotorB, followerMotorC;

  private double shooterReference;
  private ControlType shooterType;
  private boolean shooting;

  private final Debouncer shooterDebouncer = new Debouncer(0.05);

  public DrumIOSpark() {
    leaderMotor = new SparkMax(DrumConstants.kLeaderCanId, MotorType.kBrushless);
    followerMotorA = new SparkMax(DrumConstants.kFollowerACanId, MotorType.kBrushless);
    followerMotorB = new SparkMax(DrumConstants.kFollowerBCanId, MotorType.kBrushless);
    followerMotorC = new SparkMax(DrumConstants.kFollowerCCanId, MotorType.kBrushless);

    leaderEncoder = leaderMotor.getEncoder();
    leaderController = leaderMotor.getClosedLoopController();

    leaderMotor.configure(
        DrumConfig.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerMotorA.configure(
        DrumConfig.followerConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotorB.configure(
        DrumConfig.followerConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotorC.configure(
        DrumConfig.followerConfigC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterType = ControlType.kVelocity;
    shooting = false;
  }

  @Override
  public void updateInputs(DrumIOInputs inputs) {
    inputs.shooterReference = Units.radiansToDegrees(getReference());
    inputs.readyToShoot = isReady();

    // Leader — used for control and logging
    inputs.leaderCurrent = leaderMotor.getOutputCurrent();
    inputs.leaderVoltage = leaderMotor.getBusVoltage() * leaderMotor.getAppliedOutput();
    inputs.leaderVelocity = Units.radiansToDegrees(leaderEncoder.getVelocity());
    inputs.leaderTemp = Fahrenheit.convertFrom(leaderMotor.getMotorTemperature(), Celsius);

    // Followers — current and temp only for diagnostics
    inputs.followerACurrent = followerMotorA.getOutputCurrent();
    inputs.followerATemp = Fahrenheit.convertFrom(followerMotorA.getMotorTemperature(), Celsius);

    inputs.followerBCurrent = followerMotorB.getOutputCurrent();
    inputs.followerBTemp = Fahrenheit.convertFrom(followerMotorB.getMotorTemperature(), Celsius);

    inputs.followerCCurrent = followerMotorC.getOutputCurrent();
    inputs.followerCTemp = Fahrenheit.convertFrom(followerMotorC.getMotorTemperature(), Celsius);
  }

  @Override
  public double getReference() {
    return shooterReference;
  }

  @Override
  public double getVelocity() {
    return leaderEncoder.getVelocity();
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
    return shooterDebouncer.calculate(
        Math.abs(leaderEncoder.getVelocity() - getReference()) < DrumConstants.kStartOnTargetVel);
  }

  @Override
  public void startShooting() {
    shooting = true;
  }

  @Override
  public void stopShooting() {
    shooting = false;
    setReference(0.0);
  }

  @Override
  public void periodic() {
    double ff = 0;

    if (shooterType == ControlType.kVelocity) {
      ff = DrumConstants.kS + (DrumConstants.kV * getReference());
    }

    if (shooting) {
      ff += 1.0;
    }

    // Followers mirror the leader automatically — only need to command the leader
    if (shooterReference > 0) {
      leaderController.setSetpoint(shooterReference, shooterType, ClosedLoopSlot.kSlot0, ff);
    } else {
      leaderController.setSetpoint(DrumConstants.kIdleVolts, ControlType.kVoltage);
    }
  }
}
