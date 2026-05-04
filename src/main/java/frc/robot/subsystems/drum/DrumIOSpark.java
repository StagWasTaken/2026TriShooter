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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.utils.LoggedTunableNumber;

public class DrumIOSpark implements DrumIO {
  private final SparkMax topLeft;
  private final SparkMax bottomLeft, topRight, bottomRight;

  private final RelativeEncoder topLeftEncoder;
  private final RelativeEncoder bottomLeftEncoder;
  private final RelativeEncoder topRightEncoder;
  private final RelativeEncoder bottomRightEncoder;

  // Normal ramp — used when actively shooting
  private final TrapezoidProfile shooterProfile;
  // Pre-rev ramp — slower accel to bring drum up to speed without brownout risk
  private final TrapezoidProfile preRevProfile;

  private TrapezoidProfile.State profileGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State();

  private boolean preRev = false;

  // PID state
  private double prevError = 0.0;
  private double lastTimestamp = 0.0;

  private double shooterReference;
  private boolean voltageMode = false;

  private LoggedTunableNumber kS, kV, kP, kD;
  private final Debouncer drumDebouncer = new Debouncer(0.05);
  public boolean shooting;

  public DrumIOSpark() {
    topLeft = new SparkMax(DrumConstants.kTopLeftCanId, MotorType.kBrushless);
    bottomLeft = new SparkMax(DrumConstants.kBottomLeftCanId, MotorType.kBrushless);
    topRight = new SparkMax(DrumConstants.kTopRightCanId, MotorType.kBrushless);
    bottomRight = new SparkMax(DrumConstants.kBottomRightCanId, MotorType.kBrushless);

    topLeftEncoder = topLeft.getEncoder();
    bottomLeftEncoder = bottomLeft.getEncoder();
    topRightEncoder = topRight.getEncoder();
    bottomRightEncoder = bottomRight.getEncoder();

    topLeft.configure(
        DrumConfig.topLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomLeft.configure(
        DrumConfig.bottomLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    topRight.configure(
        DrumConfig.topRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(
        DrumConfig.bottomRightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                DrumConstants.kProfileMaxVel, DrumConstants.kProfileMaxAccel));

    preRevProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                DrumConstants.kPreRevMaxVel, DrumConstants.kPreRevMaxAccel));

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
  public void updateInputs(DrumIOInputs inputs, PowerDistribution pdh) {
    inputs.shooterReference = getReference();
    inputs.profiledReference = getProfiledReference();
    inputs.readyToShoot = isReady();

    inputs.topLeftCurrent = topLeft.getOutputCurrent();
    inputs.topLeftSupplyCurrent = pdh.getCurrent(topLeft.getDeviceId());
    inputs.topLeftVoltage = topLeft.getBusVoltage() * topLeft.getAppliedOutput();
    inputs.topLeftVelocity = topLeftEncoder.getVelocity();
    inputs.topLeftTemp = Fahrenheit.convertFrom(topLeft.getMotorTemperature(), Celsius);

    inputs.bottomLeftCurrent = bottomLeft.getOutputCurrent();
    inputs.bottomLeftSupplyCurrent = pdh.getCurrent(bottomLeft.getDeviceId());
    inputs.bottomLeftVoltage = bottomLeft.getBusVoltage() * bottomLeft.getAppliedOutput();
    inputs.bottomLeftTemp = Fahrenheit.convertFrom(bottomLeft.getMotorTemperature(), Celsius);
    inputs.bottomLeftVel = bottomLeftEncoder.getVelocity();

    inputs.topRightCurrent = topRight.getOutputCurrent();
    inputs.topRightSupplyCurrent = pdh.getCurrent(topRight.getDeviceId());
    inputs.topRightVoltage = topRight.getBusVoltage() * topRight.getAppliedOutput();
    inputs.topRightTemp = Fahrenheit.convertFrom(topRight.getMotorTemperature(), Celsius);
    inputs.topRightVel = topRightEncoder.getVelocity();

    inputs.bottomRightCurrent = bottomRight.getOutputCurrent();
    inputs.bottomRightSupplyCurrent = pdh.getCurrent(bottomRight.getDeviceId());
    inputs.bottomRightVoltage = bottomRight.getBusVoltage() * bottomRight.getAppliedOutput();
    inputs.bottomRightTemp = Fahrenheit.convertFrom(bottomRight.getMotorTemperature(), Celsius);
    inputs.bottomRightVel = bottomRightEncoder.getVelocity();
  }

  @Override
  public double getReference() {
    return shooterReference;
  }

  public double getProfiledReference() {
    return profileSetpoint.position;
  }

  @Override
  public double getVelocity() {
    return topLeftEncoder.getVelocity();
  }

  @Override
  public void setReference(double velocity) {
    if (voltageMode || shooterReference <= 0) {
      profileSetpoint = new TrapezoidProfile.State(topLeftEncoder.getVelocity(), 0);
    }
    shooterReference = velocity;
    profileGoal = new TrapezoidProfile.State(velocity, 0);
    voltageMode = false;
    preRev = false;
  }

  @Override
  public void setPreRev(double velocity) {
    if (voltageMode || shooterReference <= 0) {
      profileSetpoint = new TrapezoidProfile.State(topLeftEncoder.getVelocity(), 0);
    }
    shooterReference = velocity;
    profileGoal = new TrapezoidProfile.State(velocity, 0);
    voltageMode = false;
    preRev = true;
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
        Math.abs(bottomLeftEncoder.getVelocity() - getReference())
            < DrumConstants.kStartOnTargetVel);
  }

  @Override
  public void startShooting() {
    shooting = true;
  }

  @Override
  public void stopShooting() {
    shooting = false;
    setReference(0.0);
    setAllVoltage(DrumConstants.kIdleVolts);
  }

  private void setAllVoltage(double voltage) {
    double clamped = MathUtil.clamp(voltage, -12.0, 12.0);
    topLeft.setVoltage(clamped);
    bottomLeft.setVoltage(clamped);
    topRight.setVoltage(clamped);
    bottomRight.setVoltage(clamped);
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      prevError = 0.0;
      profileSetpoint = new TrapezoidProfile.State(topLeftEncoder.getVelocity(), 0);
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
      return;
    }

    // Advance whichever profile is active
    TrapezoidProfile activeProfile = preRev ? preRevProfile : shooterProfile;
    profileSetpoint = activeProfile.calculate(0.02, profileSetpoint, profileGoal);
    double profiledVel = profileSetpoint.position;

    double kSVal = Robot.tuningMode ? kS.get() : DrumConstants.kS;
    double kVVal = Robot.tuningMode ? kV.get() : DrumConstants.kV;
    double ff = kSVal * Math.signum(profiledVel) + kVVal * profiledVel;

    double kPVal = Robot.tuningMode ? kP.get() : DrumConstants.kP;
    double kDVal = Robot.tuningMode ? kD.get() : DrumConstants.kD;
    double error = profiledVel - topLeftEncoder.getVelocity();
    double dError = (error - prevError) / dt;
    prevError = error;
    double pid = kPVal * error + kDVal * dError;

    setAllVoltage(ff + pid);
  }
}
