package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.custom.pids.MapleJoystickDriveInput;
import java.util.function.DoubleSupplier;

public interface DriverMap extends Subsystem {
  Trigger povUp();

  Trigger povDown();

  Trigger povLeft();

  Trigger povRight();

  Trigger resetOdometryButton();

  Trigger stopWithXButton();

  Trigger autoAlignmentButton();

  Trigger rightBumper();

  Trigger intakeButton();

  Trigger aButton();

  Trigger bButton();

  Trigger yButton();

  Trigger scoreButton();

  Trigger autoRotationButton();

  DoubleSupplier translationalAxisX();

  DoubleSupplier translationalAxisY();

  DoubleSupplier rotationalAxisX();

  DoubleSupplier rotationalAxisY();

  CommandGenericHID getController();

  default MapleJoystickDriveInput getDriveInput() {
    return new MapleJoystickDriveInput(
        this.translationalAxisX(), this.translationalAxisY(), this.rotationalAxisX());
  }

  default Command rumble(double seconds) {
    return runEnd(
            () -> getController().setRumble(GenericHID.RumbleType.kBothRumble, 1),
            () -> getController().setRumble(GenericHID.RumbleType.kBothRumble, 0))
        .withTimeout(seconds);
  }

  default Command rumbleLeftRight(double seconds) {
    return Commands.sequence(
        runOnce(() -> getController().setRumble(GenericHID.RumbleType.kLeftRumble, 1)),
        Commands.waitSeconds(seconds),
        runOnce(
            () -> {
              getController().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
              getController().setRumble(GenericHID.RumbleType.kRightRumble, 1);
            }),
        Commands.waitSeconds(seconds),
        runOnce(() -> getController().setRumble(GenericHID.RumbleType.kBothRumble, 0)));
  }

  abstract class DriverXbox implements DriverMap {
    protected final CommandXboxController xboxController;

    protected DriverXbox(int port) {
      this.xboxController = new CommandXboxController(port);
    }

    @Override
    public Trigger povUp() {
      return xboxController.povUp();
    }

    @Override
    public Trigger povDown() {
      return xboxController.povDown();
    }

    @Override
    public Trigger povLeft() {
      return xboxController.povLeft();
    }

    @Override
    public Trigger povRight() {
      return xboxController.povRight();
    }

    @Override
    public Trigger resetOdometryButton() {
      return xboxController.start();
    }

    @Override
    public Trigger stopWithXButton() {
      return xboxController.x();
    }

    @Override
    public Trigger autoAlignmentButton() {
      return xboxController.leftBumper();
    }

    @Override
    public Trigger rightBumper() {
      return xboxController.rightBumper();
    }

    @Override
    public Trigger intakeButton() {
      return xboxController.leftTrigger(0.5);
    }

    @Override
    public Trigger aButton() {
      return xboxController.a();
    }

    @Override
    public Trigger bButton() {
      return xboxController.b();
    }

    @Override
    public Trigger yButton() {
      return xboxController.y();
    }

    @Override
    public Trigger scoreButton() {
      return xboxController.rightTrigger(0.5);
    }

    @Override
    public CommandGenericHID getController() {
      return xboxController;
    }
  }

  final class LeftHandedXbox extends DriverXbox {
    public LeftHandedXbox(int port) {
      super(port);
    }

    @Override
    public Trigger autoRotationButton() {
      return xboxController.rightStick();
    }

    @Override
    public DoubleSupplier translationalAxisX() {
      return xboxController::getLeftX;
    }

    @Override
    public DoubleSupplier translationalAxisY() {
      return xboxController::getLeftY;
    }

    @Override
    public DoubleSupplier rotationalAxisX() {
      return xboxController::getRightX;
    }

    @Override
    public DoubleSupplier rotationalAxisY() {
      return xboxController::getRightY;
    }
  }

  class RightHandedXbox extends DriverXbox {
    public RightHandedXbox(int port) {
      super(port);
    }

    @Override
    public Trigger autoRotationButton() {
      return xboxController.leftStick();
    }

    @Override
    public DoubleSupplier translationalAxisX() {
      return xboxController::getRightX;
    }

    @Override
    public DoubleSupplier translationalAxisY() {
      return xboxController::getRightY;
    }

    @Override
    public DoubleSupplier rotationalAxisX() {
      return xboxController::getLeftX;
    }

    @Override
    public DoubleSupplier rotationalAxisY() {
      return xboxController::getLeftY;
    }
  }
}
