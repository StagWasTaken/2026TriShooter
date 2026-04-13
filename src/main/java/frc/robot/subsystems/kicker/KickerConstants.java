package frc.robot.subsystems.kicker;

import frc.robot.Robot;
import frc.robot.Robot.RobotName;

public class KickerConstants {
  public static final int kKickerLeadCanId = 16;
  public static final int kKickerBottomRightanId = 18;

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kV = 0.0;
  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kA = 0.0;

  public static final double kPSim = 0.0;
  public static final double kISim = 0.0;
  public static final double kDSim = 0.0;

  public static final double kSSim = 0.0;
  public static final double kVSim = 0.0;
  public static final double kGSim = 0.0;
  public static final double kASim = 0.0;

  public static final boolean kInverted = Robot.CURRENT_ROBOT == RobotName.HYDRA ? false : true;
  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double kTolerance = Math.PI * 2;

  public static final double kKick =
      Robot.CURRENT_ROBOT == RobotName.HYDRA ? 11 /* volts */ : 7.0 /* rpm */;
  public static final double kOff = 0;

  public static final double kGearRatio = 2;
}
