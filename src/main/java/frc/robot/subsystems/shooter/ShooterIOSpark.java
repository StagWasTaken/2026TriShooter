package frc.robot.subsystems.shooter;

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

public class ShooterIOSpark implements ShooterIO {
  private final SparkMax leftShooterMotor, middleShooterMotor, rightShooterMotor;
  private final RelativeEncoder leftShooterEncoder, middleShooterEncoder, rightShooterEncoder;
  private final SparkClosedLoopController leftShooterController,
      middleShooterController,
      rightShooterController;

  private double shooterReference;
  private ControlType shooterType;
  private boolean shooting;

  private final Debouncer shooterDebouncer = new Debouncer(.05);

  // tuning
  // private final LoggedNetworkNumber leftS, leftV, middleS, middleV, rightS, rightV, reference;

  public ShooterIOSpark() {
    // initialize motor
    leftShooterMotor = new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
    middleShooterMotor = new SparkMax(ShooterConstants.kMiddleShooterCanId, MotorType.kBrushless);
    rightShooterMotor = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

    // initalize encoder
    leftShooterEncoder = leftShooterMotor.getEncoder();
    middleShooterEncoder = middleShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();

    // initialize ClosedLoop controller
    leftShooterController = leftShooterMotor.getClosedLoopController();
    middleShooterController = middleShooterMotor.getClosedLoopController();
    rightShooterController = rightShooterMotor.getClosedLoopController();

    // apply config
    leftShooterMotor.configure(
        ShooterConfig.leftShooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    middleShooterMotor.configure(
        ShooterConfig.middleShooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightShooterMotor.configure(
        ShooterConfig.rightShooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // tuning

    // leftS = new LoggedNetworkNumber("/Tuning/Shooter/leftS", ShooterConstants.kLeftShooterS);
    // leftV = new LoggedNetworkNumber("/Tuning/Shooter/leftV", ShooterConstants.kLeftShooterV);
    // middleS = new LoggedNetworkNumber("/Tuning/Shooter/middleS",
    // ShooterConstants.kMiddleShooterS);
    // middleV = new LoggedNetworkNumber("/Tuning/Shooter/middleV",
    // ShooterConstants.kMiddleShooterV);
    // rightS = new LoggedNetworkNumber("/Tuning/Shooter/rightS",
    // ShooterConstants.kRightShooterS);
    // rightV = new LoggedNetworkNumber("/Tuning/Shooter/rightV",
    // ShooterConstants.kRightShooterV);
    // reference = new LoggedNetworkNumber("/Tuning/Shooter/reference", 0.0);

    shooterType = ControlType.kVelocity;
    shooting = false;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterReference = Units.radiansToDegrees(getReference());
    inputs.readyToShoot = isReady();

    inputs.leftShooterCurrent = getLeftCurrent();
    inputs.leftShooterVoltage = getLeftVoltage();
    inputs.leftShooterVelocity = Units.radiansToDegrees(getLeftVelocity());
    inputs.leftShooterTemp =
        Fahrenheit.convertFrom(leftShooterMotor.getMotorTemperature(), Celsius);

    inputs.middleShooterCurrent = getMiddleCurrent();
    inputs.middleShooterVoltage = getMiddleVoltage();
    inputs.middleShooterVelocity = Units.radiansToDegrees(getMiddleVelocity());
    inputs.middleShooterTemp =
        Fahrenheit.convertFrom(middleShooterMotor.getMotorTemperature(), Celsius);

    inputs.rightShooterCurrent = getRightCurrent();
    inputs.rightShooterVoltage = getRightVoltage();
    inputs.rightShooterVelocity = Units.radiansToDegrees(getRightVelocity());
    inputs.rightShooterTemp =
        Fahrenheit.convertFrom(rightShooterMotor.getMotorTemperature(), Celsius);
  }

  @Override
  public double getReference() {
    return shooterReference;
  }

  @Override
  public double getLeftVelocity() {
    return leftShooterEncoder.getVelocity();
  }

  @Override
  public double getLeftCurrent() {
    return leftShooterMotor.getOutputCurrent();
  }

  @Override
  public double getLeftVoltage() {
    return leftShooterMotor.getBusVoltage() * leftShooterMotor.getAppliedOutput();
  }

  @Override
  public double getMiddleVelocity() {
    return middleShooterEncoder.getVelocity();
  }

  @Override
  public double getMiddleCurrent() {
    return middleShooterMotor.getOutputCurrent();
  }

  @Override
  public double getMiddleVoltage() {
    return middleShooterMotor.getBusVoltage() * middleShooterMotor.getAppliedOutput();
  }

  @Override
  public double getRightVelocity() {
    return rightShooterEncoder.getVelocity();
  }

  @Override
  public double getRightCurrent() {
    return rightShooterMotor.getOutputCurrent();
  }

  @Override
  public double getRightVoltage() {
    return rightShooterMotor.getBusVoltage() * rightShooterMotor.getAppliedOutput();
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
    boolean leftReady =
        Math.abs(getLeftVelocity() - getReference()) < ShooterConstants.kStartOnTargetVel;
    boolean middleReady =
        Math.abs(getMiddleVelocity() - getReference()) < ShooterConstants.kStartOnTargetVel;
    boolean rightReady =
        Math.abs(getRightVelocity() - getReference()) < ShooterConstants.kStartOnTargetVel;

    return shooterDebouncer.calculate(leftReady && middleReady && rightReady);
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
    double leftFF = 0, middleFF = 0, rightFF = 0;

    // tuning

    // double referenceRad = Units.degreesToRadians(reference.get());
    // shooterReference = referenceRad;

    // if(shooterType == ControlType.kVelocity && referenceRad != 0){
    //     leftFF =
    //         leftS.get()
    //         + (leftV.get() * referenceRad);

    //     middleFF =
    //         middleS.get()
    //         + (middleV.get() * referenceRad);

    //     rightFF =
    //         rightS.get()
    //         + (rightV.get() * referenceRad);
    // }

    // leftShooterController.setSetpoint(referenceRad, shooterType, ClosedLoopSlot.kSlot0,
    // leftFF);
    // middleShooterController.setSetpoint(referenceRad, shooterType, ClosedLoopSlot.kSlot0,
    // middleFF);
    // rightShooterController.setSetpoint(referenceRad, shooterType, ClosedLoopSlot.kSlot0,
    // rightFF);

    // real

    if (shooterType == ControlType.kVelocity) {
      leftFF = ShooterConstants.kLeftShooterS + (ShooterConstants.kLeftShooterV * getReference());

      middleFF =
          ShooterConstants.kMiddleShooterS + (ShooterConstants.kMiddleShooterV * getReference());

      rightFF =
          ShooterConstants.kRightShooterS + (ShooterConstants.kRightShooterV * getReference());
    }

    if (shooting) {
      leftFF += 1.0;
      middleFF += 1.0;
      rightFF += 1.0;
    }

    // Bypass velocity control at 0 RPM to prevent chatter and allow a smooth coast-down
    if (shooterReference > 0) {
      leftShooterController.setSetpoint(
          shooterReference, shooterType, ClosedLoopSlot.kSlot0, leftFF);
      middleShooterController.setSetpoint(
          shooterReference, shooterType, ClosedLoopSlot.kSlot0, middleFF);
      rightShooterController.setSetpoint(
          shooterReference, shooterType, ClosedLoopSlot.kSlot0, rightFF);
    } else {
      leftShooterController.setSetpoint(ShooterConstants.kIdleVolts, ControlType.kVoltage);
      middleShooterController.setSetpoint(ShooterConstants.kIdleVolts, ControlType.kVoltage);
      rightShooterController.setSetpoint(ShooterConstants.kIdleVolts, ControlType.kVoltage);
    }
  }
}
