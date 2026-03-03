package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.Vision_Constants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private static final double MAX_DISTANCE_METERS = 3.0;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      if (DriverStation.isTeleopEnabled()) {

        if (result.multitagResult.isPresent()) {
          var multitagResult = result.multitagResult.get();
          List<PhotonTrackedTarget> validTargets = new ArrayList<>();
          double totalDistance = 0.0;

          for (var target : result.targets) {
            double dist = target.bestCameraToTarget.getTranslation().getNorm();
            if (dist <= MAX_DISTANCE_METERS) {
              validTargets.add(target);
              totalDistance += dist;
            }
          }

          if (!validTargets.isEmpty()) {
            Transform3d fieldToCamera = multitagResult.estimatedPose.best;
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Pose3d robotPose =
                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            for (var t : validTargets) tagIds.add((short) t.fiducialId);

            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(),
                    robotPose,
                    multitagResult.estimatedPose.ambiguity,
                    validTargets.size(),
                    totalDistance / validTargets.size(),
                    PoseObservationType.PHOTONVISION));
          }

        } else if (!result.targets.isEmpty()) {
          var target = result.targets.get(0);
          double distance = target.bestCameraToTarget.getTranslation().getNorm();

          if (distance <= MAX_DISTANCE_METERS) {
            var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
              Transform3d fieldToTarget =
                  new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
              Transform3d cameraToTarget = target.bestCameraToTarget;
              Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
              Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
              Pose3d robotPose =
                  new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

              tagIds.add((short) target.fiducialId);

              poseObservations.add(
                  new PoseObservation(
                      result.getTimestampSeconds(),
                      robotPose,
                      target.poseAmbiguity,
                      1,
                      distance,
                      PoseObservationType.PHOTONVISION));
            }
          }
        }
      }
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Short::intValue).toArray();
  }
}
