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

  private final SparkMax intakeExtenderMotor;
  private final AbsoluteEncoder intakeExtenderEncoder;
  private final SparkClosedLoopController intakeExtenderController;

  // Roller profiles
  private final TrapezoidProfile intakeProfile;
  private TrapezoidProfile.State rollerSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State rollerGoal = new TrapezoidProfile.State();

  // Extender profiles (Changed to allow switching)
  private TrapezoidProfile extenderProfile;
  private TrapezoidProfile.Constraints extenderConstraints;
  private TrapezoidProfile.State extenderSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();

  private double topPrevError = 0.0;
  private double bottomPrevError = 0.0;
  private double lastTimestamp = 0.0;

  private double intakeReference;
  private boolean voltageMode = false;

  private double intakeExtenderReference;
  private ControlType intakeExtenderType;
  private double extenderOffsetRad = 0.0;

  private LoggedTunableNumber kS, kV, kP, kD;
  private LoggedTunableNumber extenderKs, extenderKv, extenderKg, extenderKp, extenderKd;
  private double extenderPrevError = 0.0;

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

    intakeProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                IntakeConstants.kProfileMaxVel, IntakeConstants.kProfileMaxAccel));

    // Initialize Extender Profile with default constants
    extenderConstraints =
        new TrapezoidProfile.Constraints(ExtenderConstants.kMaxVel, ExtenderConstants.kMaxAccel);
    extenderProfile = new TrapezoidProfile(extenderConstraints);

    intakeReference = 0.0;
    voltageMode = false;

    intakeExtenderReference = ExtenderConstants.kStow;
    intakeExtenderType = ControlType.kMAXMotionPositionControl;

    if (Robot.tuningMode) {
      kS = new LoggedTunableNumber("Intake/kS", IntakeConstants.kS);
      kV = new LoggedTunableNumber("Intake/kV", IntakeConstants.kV);
      kP = new LoggedTunableNumber("Intake/kP", IntakeConstants.kP);
      kD = new LoggedTunableNumber("Intake/kD", IntakeConstants.kD);

      extenderKs = new LoggedTunableNumber("Intake/Extender/kS", ExtenderConstants.kS);
      extenderKv = new LoggedTunableNumber("Intake/Extender/kV", ExtenderConstants.kV);
      extenderKg = new LoggedTunableNumber("Intake/Extender/kG", ExtenderConstants.kG);
      extenderKp = new LoggedTunableNumber("Intake/Extender/kP", ExtenderConstants.kP);
      extenderKd = new LoggedTunableNumber("Intake/Extender/kD", ExtenderConstants.kD);
    }
  }

  /**
   * THE SWITCH: Call this to change the speed/acceleration of the arm. You should add this method
   * signature to your IntakeIO interface as well.
   */
  @Override
  public void setExtenderProfileConstraints(double maxVel, double maxAccel) {
    if (maxVel != extenderConstraints.maxVelocity
        || maxAccel != extenderConstraints.maxAcceleration) {
      extenderConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
      extenderProfile = new TrapezoidProfile(extenderConstraints);
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeReference = intakeReference;
    inputs.intakeVelocity = intakeEncoder.getVelocity();
    inputs.intakeVoltage = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    inputs.intakeCurrent = intakeMotor.getOutputCurrent();

    inputs.intakeBottomVelocity = intakeSecondaryEncoder.getVelocity();
    inputs.intakeBottomVoltage =
        intakeSecondaryMotor.getBusVoltage() * intakeSecondaryMotor.getAppliedOutput();
    inputs.intakeBottomCurrent = intakeSecondaryMotor.getOutputCurrent();

    if (intakeExtenderType == ControlType.kMAXMotionPositionControl) {
      inputs.extenderReference = Units.radiansToDegrees(intakeExtenderReference);
    } else {
      inputs.extenderReference = intakeExtenderReference;
    }

    inputs.extenderProfilePositionSetpoint = Units.radiansToDegrees(extenderSetpoint.position);
    inputs.extenderProfileVelocitySetpoint = Units.radiansToDegrees(extenderSetpoint.velocity);

    inputs.extenderVoltage =
        intakeExtenderMotor.getBusVoltage() * intakeExtenderMotor.getAppliedOutput();
    inputs.extenderVelocity = intakeExtenderEncoder.getVelocity();
    inputs.extenderCurrent = intakeExtenderMotor.getOutputCurrent();
    inputs.extenderPosition = Units.radiansToDegrees(getExtenderPosition());
    inputs.extenderInPosition = getExtenderInPosition();

    inputs.intakeTemp = Fahrenheit.convertFrom(intakeMotor.getMotorTemperature(), Celsius);
    inputs.intakeFollowerTemp =
        Fahrenheit.convertFrom(intakeSecondaryMotor.getMotorTemperature(), Celsius);
    inputs.extenderTemp =
        Fahrenheit.convertFrom(intakeExtenderMotor.getMotorTemperature(), Celsius);
  }

  @Override
  public void setReference(double velocity) {
    if (voltageMode || intakeReference <= 0) {
      rollerSetpoint = new TrapezoidProfile.State(intakeEncoder.getVelocity(), 0);
    }
    intakeReference = velocity;
    rollerGoal = new TrapezoidProfile.State(velocity, 0);
    voltageMode = false;
  }

  @Override
  public void setVoltage(double voltage) {
    intakeReference = voltage;
    voltageMode = true;
  }

  @Override
  public void setExtenderReference(double angRad) {
    if (angRad == intakeExtenderReference && intakeExtenderType != ControlType.kVoltage) return;
    if (intakeExtenderType == ControlType.kVoltage) {
      extenderSetpoint = new TrapezoidProfile.State(getExtenderPosition(), 0);
    }
    intakeExtenderReference = angRad;
    extenderGoal = new TrapezoidProfile.State(angRad, 0);
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
    extenderSetpoint = new TrapezoidProfile.State(getExtenderPosition(), 0);
  }

  @Override
  public double getExtenderVelocity() {
    return intakeExtenderEncoder.getVelocity();
  }

  @Override
  public boolean getExtenderInPosition() {
    if (intakeExtenderType == ControlType.kVoltage) return false;
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
    double targetRad = ExtenderConstants.kExtended;
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
      extenderPrevError = 0.0;
      rollerSetpoint = new TrapezoidProfile.State(intakeEncoder.getVelocity(), 0);
      lastTimestamp = Timer.getFPGATimestamp();

      intakeExtenderController.setSetpoint(
          intakeExtenderReference, intakeExtenderType, ClosedLoopSlot.kSlot0);

      extenderSetpoint = new TrapezoidProfile.State(getExtenderPosition(), 0);
      extenderGoal = new TrapezoidProfile.State(getExtenderPosition(), 0);
      return;
    }

    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;
    if (dt <= 0.0 || dt > 0.5) dt = 0.02;

    // Extender Logic - Automatically uses whatever 'extenderProfile' is currently active
    if (intakeExtenderType == ControlType.kVoltage) {
      intakeExtenderMotor.setVoltage(MathUtil.clamp(intakeExtenderReference, -12.0, 12.0));
    } else {
      extenderSetpoint = extenderProfile.calculate(0.02, extenderSetpoint, extenderGoal);

      double kSVal = Robot.tuningMode ? extenderKs.get() : ExtenderConstants.kS;
      double kVVal = Robot.tuningMode ? extenderKv.get() : ExtenderConstants.kV;
      double kGVal = Robot.tuningMode ? extenderKg.get() : ExtenderConstants.kG;
      double kPVal = Robot.tuningMode ? extenderKp.get() : ExtenderConstants.kP;
      double kDVal = Robot.tuningMode ? extenderKd.get() : ExtenderConstants.kD;

      double curPos = getExtenderPosition();

      double gravityFF = kGVal * Math.sin(curPos);
      double velocityFF = kVVal * extenderSetpoint.velocity;
      double staticFF = kSVal * Math.signum(extenderSetpoint.velocity);

      double error = extenderSetpoint.position - curPos;
      double dError = (error - extenderPrevError) / dt;
      extenderPrevError = error;

      double output = gravityFF + velocityFF + staticFF + kPVal * error + kDVal * dError;
      intakeExtenderMotor.setVoltage(MathUtil.clamp(output, -12.0, 12.0));
    }

    // Roller Logic
    if (voltageMode) {
      setTopVoltage(intakeReference);
      setBottomVoltage(intakeReference);
    } else if (intakeReference <= 0) {
      rollerSetpoint = new TrapezoidProfile.State();
      rollerGoal = new TrapezoidProfile.State();
      topPrevError = 0.0;
      bottomPrevError = 0.0;
      setTopVoltage(0);
      setBottomVoltage(0);
    } else {
      rollerSetpoint = intakeProfile.calculate(0.02, rollerSetpoint, rollerGoal);
      double targetVel = rollerSetpoint.position;

      double kSVal = Robot.tuningMode ? kS.get() : IntakeConstants.kS;
      double kVVal = Robot.tuningMode ? kV.get() : IntakeConstants.kV;
      double kPVal = Robot.tuningMode ? kP.get() : IntakeConstants.kP;
      double kDVal = Robot.tuningMode ? kD.get() : IntakeConstants.kD;

      double ff = kSVal * Math.signum(targetVel) + kVVal * targetVel;

      double topError = targetVel - intakeEncoder.getVelocity();
      double topDError = (topError - topPrevError) / dt;
      topPrevError = topError;
      setTopVoltage(ff + kPVal * topError + kDVal * topDError);

      double bottomError = targetVel - intakeSecondaryEncoder.getVelocity();
      double bottomDError = (bottomError - bottomPrevError) / dt;
      bottomPrevError = bottomError;
      setBottomVoltage(ff + kPVal * bottomError + kDVal * bottomDError);
    }
  }
}
