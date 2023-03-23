package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    public PhotonCameraWrapper() {
        // Set up a test arena of two apriltags at the center of each driver station ses

        // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
        AprilTagFieldLayout atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

        // Forward Camera
        photonCamera =
                new PhotonCamera(
                        VisionConstants
                                .cameraName); // Change the name of your camera here to whatever it is in the
        // PhotonVision UI.

        // Create pose estimator
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.robotToCam);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}
