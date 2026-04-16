package frc.robot.utils.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
  public static final double FIELD_WIDTH = 16.541; // 651.2 inches (length, X axis)
  public static final double FIELD_HEIGHT = 8.072; // 317.7 inches (width, Y axis)

  public static final Translation2d BlueHubPose =
      new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
  public static final Translation2d RedHubPose =
      new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));

  // corner of the field on blue side
  public static final Translation2d RightPassingTarget = new Translation2d(1.5, 2.5);
  public static final Translation2d LeftPassingTarget = new Translation2d(1.5, 5.5);

  // Returns the current alliance, defaulting to Blue if disconnected
  public static DriverStation.Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
  }

  // Dynamically picks the pose based on the current alliance
  public static Translation2d getHubPose() {
    return (getAlliance() == DriverStation.Alliance.Blue)
        ? FieldConstants.BlueHubPose
        : FieldConstants.RedHubPose;
  }
}
