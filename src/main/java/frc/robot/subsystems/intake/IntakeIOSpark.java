package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.utils.LoggedTunableNumber;

public class IntakeIOSpark implements IntakeIO {
  private final SparkFlex intakeMotor;
  private final SparkFlex intakeSecondaryMotor;
  private final RelativeEncoder intakeEncoder;
  private final RelativeEncoder intakeSecondaryEncoder;

  // Top roller profile — position axis = RPM, velocity axis = RPM/s
  private final TrapezoidProfile topProfile;
  private TrapezoidProfile.State topProfileGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State topProfileSetpoint = new TrapezoidProfile.State();

  // Bottom roller profile
  private final TrapezoidProfile bottomProfile;
  private TrapezoidProfile.State bottomProfileGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State bottomProfileSetpoint = new TrapezoidProfile.State();

  // PID state — top
  private double topPrevError = 0.0;

  // PID state — bottom
  private double bottomPrevError = 0.0;

  private double lastTimestamp = 0.0;

  private double intakeReference;
  private boolean voltageMode = false;

  private LoggedTunableNumber kS, kV, kP, kD;
  private LoggedTunableNumber kSBottom, kVBottom, kPBottom, kDBottom;

  private final SparkMax intakeExtenderMotor;
  private final AbsoluteEncoder intakeExtenderEncoder;
  private final SparkClosedLoopController intakeExtenderController;

  private double intakeExtenderReference;
  private ControlType intakeExtenderType;
  private double extenderOffsetRad = 0.0;

  public IntakeIOSpark() {
    intakeMotor = new SparkFlex(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    intakeSecondaryMotor =
        new SparkFlex(IntakeConstants.kIntakeBottomRightanId, MotorType.kBrushless);

    intakeExtenderMotor =
        new SparkMax(ExtenderConstants.kIntakeExtenderCanId, MotorType.kBrushless);

    intakeExtenderController = intakeExtenderMotor.getClosedLoopController();

    intakeEncoder = intakeMotor.getEncoder();
    intakeSecondaryEncoder = intakeSecondaryMotor.getEncoder();
    intakeExtenderEncoder = intakeExtenderMotor.getAbsoluteEncoder();

    intakeMotor.configure(
        IntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeSecondaryMotor.configure(
        IntakeConfig.secondaryRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeExtenderMotor.configure(
        IntakeConfig.intakeExtenderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    topProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                IntakeConstants.kProfileMaxVel, IntakeConstants.kProfileMaxAccel));
    bottomProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                IntakeConstants.kProfileMaxVel, IntakeConstants.kProfileMaxAccel));

    intakeReference = 0.0;
    voltageMode = false;

    intakeExtenderReference = ExtenderConstants.kStow;
    intakeExtenderType = ControlType.kMAXMotionPositionControl;

    if (Robot.tuningMode) {
      kS = new LoggedTunableNumber("Intake/Top/kS", IntakeConstants.kS);
      kV = new LoggedTunableNumber("Intake/Top/kV", IntakeConstants.kV);
      kP = new LoggedTunableNumber("Intake/Top/kP", IntakeConstants.kP);
      kD = new LoggedTunableNumber("Intake/Top/kD", IntakeConstants.kD);

      kSBottom = new LoggedTunableNumber("Intake/Bottom/kS", IntakeConstants.kSBottom);
      kVBottom = new LoggedTunableNumber("Intake/Bottom/kV", IntakeConstants.kVBottom);
      kPBottom = new LoggedTunableNumber("Intake/Bottom/kP", IntakeConstants.kPBottom);
      kDBottom = new LoggedTunableNumber("Intake/Bottom/kD", IntakeConstants.kDBottom);
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeReference = getReference();
    inputs.intakeProfileSetpoint = topProfileSetpoint.position;
    inputs.intakeBottomProfileSetpoint = bottomProfileSetpoint.position;
    inputs.intakeCurrent = getCurrent();
    inputs.intakeVoltage = getVoltage();
    inputs.intakeVelocity = getVelocity();
    inputs.intakeSpeedFtPerSec = getVelocity() * IntakeConstants.kRPMToFtPerSec;
    inputs.intakeBottomCurrent = intakeSecondaryMotor.getOutputCurrent();
    inputs.intakeBottomVoltage =
        intakeSecondaryMotor.getAppliedOutput() * intakeSecondaryMotor.getBusVoltage();
    inputs.intakeBottomVelocity = intakeSecondaryEncoder.getVelocity();
    inputs.intakeBottomSpeedFtPerSec =
        intakeSecondaryEncoder.getVelocity() * IntakeConstants.kBottomRPMToFtPerSec;
    inputs.intakePosition = getPosition();
    inputs.intakeTemp = Fahrenheit.convertFrom(intakeMotor.getMotorTemperature(), Celsius);
    inputs.intakeFollowerTemp =
        Fahrenheit.convertFrom(intakeSecondaryMotor.getMotorTemperature(), Celsius);

    inputs.extenderReference = Units.radiansToDegrees(getExtenderReference());
    inputs.extenderCurrent = getExtenderCurrent();
    inputs.extenderVoltage = getExtenderVoltage();
    inputs.extenderVelocity = getExtenderVelocity();
    inputs.extenderPosition = Units.radiansToDegrees(getExtenderPosition());
    inputs.extenderInPosition = getExtenderInPosition();
    inputs.extenderTemp =
        Fahrenheit.convertFrom(intakeExtenderMotor.getMotorTemperature(), Celsius);
  }

  @Override
  public void setReference(double velocity) {
    if (voltageMode || intakeReference <= 0) {
      topProfileSetpoint = new TrapezoidProfile.State(intakeEncoder.getVelocity(), 0);
      bottomProfileSetpoint = new TrapezoidProfile.State(intakeSecondaryEncoder.getVelocity(), 0);
    }
    intakeReference = velocity;
    topProfileGoal = new TrapezoidProfile.State(velocity, 0);
    bottomProfileGoal = new TrapezoidProfile.State(velocity - 500, 0);
    voltageMode = false;
  }

  @Override
  public double getReference() {
    return intakeReference;
  }

  @Override
  public void setVoltage(double voltage) {
    intakeReference = voltage;
    voltageMode = true;
    topPrevError = 0.0;
    bottomPrevError = 0.0;
    topProfileSetpoint = new TrapezoidProfile.State();
    topProfileGoal = new TrapezoidProfile.State();
    bottomProfileSetpoint = new TrapezoidProfile.State();
    bottomProfileGoal = new TrapezoidProfile.State();
  }

  @Override
  public double getVoltage() {
    return intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
  }

  @Override
  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public double getCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  @Override
  public double getPosition() {
    return intakeEncoder.getPosition();
  }

  @Override
  public void setExtenderReference(double angRad) {
    intakeExtenderReference = angRad;
    intakeExtenderType = ControlType.kMAXMotionPositionControl;
  }

  @Override
  public double getExtenderReference() {
    return intakeExtenderReference;
  }

  @Override
  public void setExtenderVoltage(double voltage) {
    intakeExtenderReference = voltage;
    intakeExtenderType = ControlType.kVoltage;
  }

  @Override
  public double getExtenderVoltage() {
    return intakeExtenderMotor.getBusVoltage() * intakeExtenderMotor.getAppliedOutput();
  }

  @Override
  public double getExtenderVelocity() {
    return intakeExtenderEncoder.getVelocity();
  }

  @Override
  public double getExtenderCurrent() {
    return intakeExtenderMotor.getOutputCurrent();
  }

  @Override
  public boolean getExtenderInPosition() {
    double positionError = Math.abs(getExtenderPosition() - getExtenderReference());
    return positionError < ExtenderConstants.kPositionTolerance;
  }

  @Override
  public double getExtenderPosition() {
    return intakeExtenderEncoder.getPosition() - extenderOffsetRad;
  }

  @Override
  public void resetEncoder() {
    double currentPhysicalPos = intakeExtenderEncoder.getPosition();
    double targetRad = Units.degreesToRadians(315.0);

    // Offset = Physical Position - Desired Position
    // This makes: (Physical - Offset) = targetRad
    extenderOffsetRad = currentPhysicalPos - targetRad;
  }

  private void setTopVoltage(double voltage) {
    intakeMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  private void setBottomVoltage(double voltage) {
    intakeSecondaryMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      topPrevError = 0.0;
      bottomPrevError = 0.0;
      topProfileSetpoint = new TrapezoidProfile.State(intakeEncoder.getVelocity(), 0);
      bottomProfileSetpoint = new TrapezoidProfile.State(intakeSecondaryEncoder.getVelocity(), 0);
      lastTimestamp = Timer.getFPGATimestamp();
      intakeExtenderController.setSetpoint(
          intakeExtenderReference, intakeExtenderType, ClosedLoopSlot.kSlot0);
      return;
    }

    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;
    if (dt <= 0.0 || dt > 0.5) dt = 0.02;

    if (voltageMode) {
      setTopVoltage(intakeReference);
      setBottomVoltage(intakeReference);
    } else if (intakeReference <= 0) {
      topProfileSetpoint = new TrapezoidProfile.State();
      topProfileGoal = new TrapezoidProfile.State();
      bottomProfileSetpoint = new TrapezoidProfile.State();
      bottomProfileGoal = new TrapezoidProfile.State();
      topPrevError = 0.0;
      bottomPrevError = 0.0;
      setTopVoltage(0);
      setBottomVoltage(0);
    } else {
      // Top roller
      topProfileSetpoint = topProfile.calculate(0.02, topProfileSetpoint, topProfileGoal);
      double topProfiledVel = topProfileSetpoint.position;

      double kSVal = Robot.tuningMode ? kS.get() : IntakeConstants.kS;
      double kVVal = Robot.tuningMode ? kV.get() : IntakeConstants.kV;
      double topFF = kSVal * Math.signum(topProfiledVel) + kVVal * topProfiledVel;

      double kPVal = Robot.tuningMode ? kP.get() : IntakeConstants.kP;
      double kDVal = Robot.tuningMode ? kD.get() : IntakeConstants.kD;
      double topError = topProfiledVel - intakeEncoder.getVelocity();
      double topDError = (topError - topPrevError) / dt;
      topPrevError = topError;

      setTopVoltage(topFF + kPVal * topError + kDVal * topDError);

      // Bottom roller
      bottomProfileSetpoint =
          bottomProfile.calculate(0.02, bottomProfileSetpoint, bottomProfileGoal);
      double bottomProfiledVel = bottomProfileSetpoint.position;

      double kSBVal = Robot.tuningMode ? kSBottom.get() : IntakeConstants.kSBottom;
      double kVBVal = Robot.tuningMode ? kVBottom.get() : IntakeConstants.kVBottom;
      double bottomFF = kSBVal * Math.signum(bottomProfiledVel) + kVBVal * bottomProfiledVel;

      double kPBVal = Robot.tuningMode ? kPBottom.get() : IntakeConstants.kPBottom;
      double kDBVal = Robot.tuningMode ? kDBottom.get() : IntakeConstants.kDBottom;
      double bottomError = bottomProfiledVel - intakeSecondaryEncoder.getVelocity();
      double bottomDError = (bottomError - bottomPrevError) / dt;
      bottomPrevError = bottomError;

      setBottomVoltage(bottomFF + kPBVal * bottomError + kDBVal * bottomDError);
    }

    // Extender always runs regardless of roller mode
    intakeExtenderController.setSetpoint(
        intakeExtenderReference, intakeExtenderType, ClosedLoopSlot.kSlot0);
  }
}
