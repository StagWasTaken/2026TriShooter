package frc.robot.subsystems.drum;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.utils.LoggedTunableNumber;

public class DrumIOSpark implements DrumIO {
  private final SparkMax topLeftLeader;
  private final SparkMax bottomLeftFollower, topRightFollower, bottomRightFollower;

  private final RelativeEncoder topLeftLeaderEncoder;
  private final RelativeEncoder bottomLeftFollowerEncoder;
  private final RelativeEncoder topRightFollowerEncoder;
  private final RelativeEncoder bottomRightFollowerEncoder;

  // Trapezoidal profile — position axis = RPM, velocity axis = RPM/s
  private final TrapezoidProfile shooterProfile;
  private TrapezoidProfile.State profileGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State();

  // PID state
  private double prevError = 0.0;
  private double lastTimestamp = 0.0;

  private double shooterReference;
  private boolean voltageMode = false;

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

    shooterProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                DrumConstants.kProfileMaxVel, DrumConstants.kProfileMaxAccel));

    shooterReference = 0.0;
    voltageMode = false;
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
    inputs.shooterReference = getReference();
    inputs.profiledReference = getProfiledReference();
    inputs.readyToShoot = isReady();

    inputs.leaderCurrent = topLeftLeader.getOutputCurrent();
    inputs.leaderVoltage = topLeftLeader.getBusVoltage() * topLeftLeader.getAppliedOutput();
    inputs.leaderVelocity = topLeftLeaderEncoder.getVelocity();
    inputs.leaderTemp = Fahrenheit.convertFrom(topLeftLeader.getMotorTemperature(), Celsius);

    inputs.followerACurrent = bottomLeftFollower.getOutputCurrent();
    inputs.followerAVoltage =
        bottomLeftFollower.getBusVoltage() * bottomLeftFollower.getAppliedOutput();
    inputs.followerATemp =
        Fahrenheit.convertFrom(bottomLeftFollower.getMotorTemperature(), Celsius);
    inputs.followerAVel = bottomLeftFollowerEncoder.getVelocity();

    inputs.followerBCurrent = topRightFollower.getOutputCurrent();
    inputs.followerBVoltage =
        topRightFollower.getBusVoltage() * topRightFollower.getAppliedOutput();
    inputs.followerBTemp = Fahrenheit.convertFrom(topRightFollower.getMotorTemperature(), Celsius);
    inputs.followerBVel = topRightFollowerEncoder.getVelocity();

    inputs.followerCCurrent = bottomRightFollower.getOutputCurrent();
    inputs.followerCVoltage =
        bottomRightFollower.getBusVoltage() * bottomRightFollower.getAppliedOutput();
    inputs.followerCTemp =
        Fahrenheit.convertFrom(bottomRightFollower.getMotorTemperature(), Celsius);
    inputs.followerCVel = bottomRightFollowerEncoder.getVelocity();
  }

  @Override
  public double getReference() {
    return shooterReference;
  }

  // Returns the current profiled RPM being commanded
  public double getProfiledReference() {
    return profileSetpoint.position;
  }

  @Override
  public double getVelocity() {
    return topLeftLeaderEncoder.getVelocity();
  }

  @Override
  public void setReference(double velocity) {
    if (velocity != shooterReference) {
      profileSetpoint = new TrapezoidProfile.State(topLeftLeaderEncoder.getVelocity(), 0);
      profileGoal = new TrapezoidProfile.State(velocity, 0);
      shooterReference = velocity;
    }
    voltageMode = false;
  }

  @Override
  public void setVoltage(double voltage) {
    shooterReference = voltage;
    voltageMode = true;
    prevError = 0.0;
    profileSetpoint = new TrapezoidProfile.State();
    profileGoal = new TrapezoidProfile.State();
  }

  @Override
  public boolean isReady() {
    return drumDebouncer.calculate(
        Math.abs(bottomLeftFollowerEncoder.getVelocity() - getReference())
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

  private void setAllVoltage(double voltage) {
    double clamped = MathUtil.clamp(voltage, -12.0, 12.0);
    topLeftLeader.setVoltage(clamped);
    bottomLeftFollower.setVoltage(clamped);
    topRightFollower.setVoltage(clamped);
    bottomRightFollower.setVoltage(clamped);
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      prevError = 0.0;
      profileSetpoint = new TrapezoidProfile.State(topLeftLeaderEncoder.getVelocity(), 0);
      lastTimestamp = Timer.getFPGATimestamp();
      return;
    }

    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;
    if (dt <= 0.0 || dt > 0.5) dt = 0.02;

    if (voltageMode) {
      setAllVoltage(shooterReference);
      return;
    }

    if (shooterReference <= 0) {
      profileSetpoint = new TrapezoidProfile.State();
      profileGoal = new TrapezoidProfile.State();
      prevError = 0.0;
      setAllVoltage(DrumConstants.kIdleVolts);
      return;
    }

    // Advance the profile — position = RPM, velocity = RPM/s
    profileSetpoint = shooterProfile.calculate(0.02, profileSetpoint, profileGoal);
    double profiledVel = profileSetpoint.position;

    // Feedforward
    double kSVal = Robot.tuningMode ? kS.get() : DrumConstants.kS;
    double kVVal = Robot.tuningMode ? kV.get() : DrumConstants.kV;
    double ff = kSVal * Math.signum(profiledVel) + kVVal * profiledVel;

    // PD on actual velocity vs profiled setpoint
    double kPVal = Robot.tuningMode ? kP.get() : DrumConstants.kP;
    double kDVal = Robot.tuningMode ? kD.get() : DrumConstants.kD;
    double error = profiledVel - topLeftLeaderEncoder.getVelocity();
    double dError = (error - prevError) / dt;
    prevError = error;
    double pid = kPVal * error + kDVal * dError;

    setAllVoltage(ff + pid);
  }
}
