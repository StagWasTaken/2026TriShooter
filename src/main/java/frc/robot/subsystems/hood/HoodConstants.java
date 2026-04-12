package frc.robot.subsystems.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Robot.RobotName;

public class HoodConstants {
  public static final int kHoodCanId = 2;

  public static final double kP = Robot.CURRENT_ROBOT == RobotName.COMP_BOT ? 5 : 0.25;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kV = 0.0;
  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kA = 0.0;

  public static final double kPSim = 20;
  public static final double kDSim = 0.0;

  public static final boolean kInverted = Robot.CURRENT_ROBOT == RobotName.COMP_BOT ? false : false;
  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double kMinPos = Robot.CURRENT_ROBOT == RobotName.COMP_BOT ? 0.025 : 5;
  public static final double kMaxPos = Robot.CURRENT_ROBOT == RobotName.COMP_BOT ? 0.95 : 50;
  ;
  public static final double kTolerance = 1;

  public static final double kMaxVel =
      Robot.CURRENT_ROBOT == RobotName.COMP_BOT ? 1 : Math.toRadians(360);
  public static final double kMaxAccel =
      Robot.CURRENT_ROBOT == RobotName.COMP_BOT ? 3 : Math.toRadians(1080);

  public static final double kGearRatio = 25;

  public static final double kHomingVoltage = -1.0;
  public static final double kHomingVelocityThreshold = 5;

  // sim
  public static final double kHoodMOI = 0.0145;
  public static final double kMinHoodAngle = Math.toRadians(53);
  public static final double kMaxHoodAngle = Math.toRadians(85);
  public static final double kHoodAngleDelta = kMaxHoodAngle - kMinHoodAngle;
  public static final double kStartHoodAngle = kMinHoodAngle;

  // 5 inch gap between flywheel and hood, 2 inch radius flywheel, and 1 inch thick hood
  public static final double kHoodRadius = Units.inchesToMeters(7.5);
}
