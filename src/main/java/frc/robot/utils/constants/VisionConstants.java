package frc.robot.utils.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class VisionConstants {
  public static final AprilTagFields CURRENT_FIELD = AprilTagFields.kDefaultField;
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(CURRENT_FIELD);
  public static final Time POSE_BUFFER_DURATION = Seconds.of(2.5);
  public static final Time ADDITIONAL_LATENCY_COMPENSATION = Milliseconds.of(40);

  // for filtering
  public static final Distance ROBOT_HEIGHT_TOLERANCE = Meters.of(0.1);
  public static final Angle ROBOT_PITCH_TOLERANCE = Degrees.of(6);
  public static final Angle ROBOT_ROLL_TOLERANCE = Degrees.of(6);
  public static final Distance MAX_TAG_DISTANCE = Meters.of(3);
  public static final double MAX_TAG_AMBIGUITY = 0.2;
  public static final Angle MAX_TAG_ANGLE = Degrees.of(70);

  /** Standard errors for single tag vision observations. */
  public static final Distance TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION =
      Meters.of(1.5);

  public static final Angle ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION =
      Degrees.of(90);

  /** Standard errors for multiple-solvePNP observations. */
  public static final Distance TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_MULTITAG = Meters.of(0.5);

  public static final Angle ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_MULTITAG = Degrees.of(8);

  /** Standard errors for focused tag observations. */
  public static final Distance TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_FOCUSED_TAG = Meters.of(0.5);

  public static final Angle ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_FOCUSED_TAG = Degrees.of(8);

  /** Odometry standard errors for the primary pose estimator */
  public static final Distance PRIMARY_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR =
      Meters.of(0.5);

  public static final Angle PRIMARY_ESTIMATOR_GYRO_ROTATIONAL_STANDARD_ERROR = Degrees.of(0.1);

  /** Odometry standard errors for the secondary (vision-sensitive) pose estimator */
  public static final Distance VISION_SENSITIVE_ESTIMATOR_ODOMETRY_TRANSLATIONAL_STANDARD_ERROR =
      Meters.of(1.5);

  public static final Angle VISION_SENSITIVE_ESTIMATOR_GYRO_ROTATIONAL_STANDARD_ERROR =
      Degrees.of(1.5);
}
