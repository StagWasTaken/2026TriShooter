package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private final RelativeEncoder intakeExtenderEncoder;

  private final TrapezoidProfile intakeProfile;
  private TrapezoidProfile.State profileGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State();

  private double topPrevError = 0.0;
  private double bottomPrevError = 0.0;
  private double lastTimestamp = 0.0;

  private double intakeReference;
  private double extenderVoltage;
  private boolean voltageMode = false;

  private LoggedTunableNumber kS, kV, kP, kD;

  public IntakeIOSpark() {
    intakeMotor = new SparkFlex(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    intakeSecondaryMotor =
        new SparkFlex(IntakeConstants.kIntakeBottomRightanId, MotorType.kBrushless);
    intakeExtenderMotor =
        new SparkMax(ExtenderConstants.kIntakeExtenderCanId, MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    intakeSecondaryEncoder = intakeSecondaryMotor.getEncoder();
    intakeExtenderEncoder = intakeExtenderMotor.getEncoder();

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

    if (Robot.tuningMode) {
      kS = new LoggedTunableNumber("Intake/kS", IntakeConstants.kS);
      kV = new LoggedTunableNumber("Intake/kV", IntakeConstants.kV);
      kP = new LoggedTunableNumber("Intake/kP", IntakeConstants.kP);
      kD = new LoggedTunableNumber("Intake/kD", IntakeConstants.kD);
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

    inputs.extenderVoltage =
        intakeExtenderMotor.getBusVoltage() * intakeExtenderMotor.getAppliedOutput();
    inputs.extenderVelocity = intakeExtenderEncoder.getVelocity();
    inputs.extenderCurrent = intakeExtenderMotor.getOutputCurrent();

    inputs.intakeTemp = Fahrenheit.convertFrom(intakeMotor.getMotorTemperature(), Celsius);
    inputs.intakeFollowerTemp =
        Fahrenheit.convertFrom(intakeSecondaryMotor.getMotorTemperature(), Celsius);
    inputs.extenderTemp =
        Fahrenheit.convertFrom(intakeExtenderMotor.getMotorTemperature(), Celsius);
  }

  @Override
  public void setReference(double velocity) {
    if (voltageMode || intakeReference <= 0) {
      profileSetpoint = new TrapezoidProfile.State(intakeEncoder.getVelocity(), 0);
    }
    intakeReference = velocity;
    profileGoal = new TrapezoidProfile.State(velocity, 0);
    voltageMode = false;
  }

  @Override
  public void setVoltage(double voltage) {
    intakeReference = voltage;
    voltageMode = true;
  }

  @Override
  public void setExtenderVoltage(double voltage) {
    this.extenderVoltage = voltage;
  }

  @Override
  public double getExtenderVelocity() {
    return intakeExtenderEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;
    if (dt <= 0.0 || dt > 0.5) dt = 0.02;

    if (RobotState.isDisabled()) {
      intakeMotor.stopMotor();
      intakeSecondaryMotor.stopMotor();
      intakeExtenderMotor.stopMotor();
      return;
    }

    intakeExtenderMotor.setVoltage(extenderVoltage);

    if (voltageMode) {
      intakeMotor.setVoltage(intakeReference);
      intakeSecondaryMotor.setVoltage(intakeReference);
    } else if (intakeReference <= 0) {
      intakeMotor.stopMotor();
      intakeSecondaryMotor.stopMotor();
    } else {
      profileSetpoint = intakeProfile.calculate(0.02, profileSetpoint, profileGoal);

      double targetVel = profileSetpoint.position;
      double kSVal = Robot.tuningMode ? kS.get() : IntakeConstants.kS;
      double kVVal = Robot.tuningMode ? kV.get() : IntakeConstants.kV;
      double kPVal = Robot.tuningMode ? kP.get() : IntakeConstants.kP;
      double kDVal = Robot.tuningMode ? kD.get() : IntakeConstants.kD;

      double ff = kSVal * Math.signum(targetVel) + kVVal * targetVel;

      double topError = targetVel - intakeEncoder.getVelocity();
      double topDError = (topError - topPrevError) / dt;
      topPrevError = topError;
      intakeMotor.setVoltage(MathUtil.clamp(ff + kPVal * topError + kDVal * topDError, -12, 12));

      double bottomError = targetVel - intakeSecondaryEncoder.getVelocity();
      double bottomDError = (bottomError - bottomPrevError) / dt;
      bottomPrevError = bottomError;
      intakeSecondaryMotor.setVoltage(
          MathUtil.clamp(ff + kPVal * bottomError + kDVal * bottomDError, -12, 12));
    }
  }
}
