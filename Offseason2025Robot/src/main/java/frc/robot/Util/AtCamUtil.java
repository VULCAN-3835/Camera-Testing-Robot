package frc.robot.Util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AtCamUtil {
    private PhotonCamera OVCamera;

    private AprilTagFieldLayout aprilTagFieldLayout;

    private Transform3d robotToCamera;

    private PhotonPipelineResult result;
    private String name;

    private PhotonPoseEstimator poseEstimator;

    public AtCamUtil(String name, Transform3d robotToCamera) {
        // constructing the camera
        this.name = name;
        this.OVCamera = new PhotonCamera(name);
        
        updateResult();
        this.OVCamera.setPipelineIndex(1);
        this.robotToCamera = robotToCamera;
        this.poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.robotToCamera);
        
        boolean fieldWork;
        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            fieldWork = true;
        } catch (Exception e) {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            fieldWork = false;
        }
        SmartDashboard.putBoolean("filed works?", fieldWork);

    }

    public boolean isConnected() {
        return this.OVCamera.isConnected();
    }

    public String getName() {
        return this.name;
    }

    private boolean hasTarget() {
        return this.result != null && this.result.hasTargets();
    }

    public boolean hasValidTarget() {
        return hasTarget() && getDistanceToTarget() < 1.5;
    }

    public void updateResult() {
        PhotonPipelineResult newResult = this.OVCamera.getAllUnreadResults().get(0);

        if (newResult != null && newResult.hasTargets()) {
            this.result = newResult;
        }
    }

    public Optional<Pose3d> getTagPose3d(int ID) {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(ID);
        return tagPose;
    }

    public int getID() {
        if (hasValidTarget()) {
            return this.result.getBestTarget().getFiducialId();
        }
        return -1;
    }

    public Optional<Double> getTargetX() {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(getID());

        if (!hasTarget() || tagPose.isEmpty()) {
            return Optional.empty();
        }

        // Get the transformation from the camera to the target
        Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();

        // Compute the X position of the target relative to the camera
        double targetX = camToTarget.getX();

        return Optional.of(targetX);
    }

    public Optional<Double> getTargetY() {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(getID());

        if (!hasTarget() || tagPose.isEmpty()) {
            return Optional.empty();
        }

        // Get the transformation from the camera to the target
        Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();

        // Compute the X position of the target relative to the camera
        double targetY = camToTarget.getY();

        return Optional.of(targetY);
    }
    
    public Optional<Angle> getTargetYaw() {
        if (hasTarget()) {
            return Optional.of(Degrees.of(result.getBestTarget().getYaw()));
        }
        return Optional.empty();
    }

    public double getDistanceToTarget() {
        if (!hasTarget()) return -1.0;

        PhotonTrackedTarget target = result.getBestTarget();
        Optional<Pose3d> tagOnField = aprilTagFieldLayout.getTagPose(target.getFiducialId());

        if (tagOnField.isPresent()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    this.robotToCamera.getZ(),               // גובה המצלמה
                    tagOnField.get().getZ(),            // גובה התג
                    this.robotToCamera.getRotation().getY(), // זווית המצלמה (Pitch)
                    Math.toRadians(target.getPitch())   // זווית המטרה בתמונה
            );
        }
        return -1.0;
    }

    public double distanceFromTargetMeters() {
        if (hasTarget()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    this.robotToCamera.getMeasureZ().in(Meters),
                    getTagPose3d(getID()).get().getMeasureZ().in(Meters),
                    this.robotToCamera.getRotation().getMeasureY().in(Radians),
                    getTagPose3d(getID()).get().getRotation().getMeasureY().in(Radians));
        }
        return -1;
    }

    public Pose2d getPoseFromCamera() {
        if (hasTarget() && aprilTagFieldLayout != null) {
            PhotonTrackedTarget bestTarget = this.result.getBestTarget();

            // Get the tag pose from the layout as Pose2d
            Optional<Pose2d> tagPoseOptional = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId())
                    .map(tagPose3D -> new Pose2d(
                            new Translation2d(tagPose3D.getTranslation().toTranslation2d().getX(),
                                    tagPose3D.getTranslation().toTranslation2d().getY()),
                            new Rotation2d(tagPose3D.getRotation().getZ())));

            if (tagPoseOptional.isPresent()) {
                Pose2d tagPose2D = tagPoseOptional.get();

                // Manually extract 2D transform values from the target
                double targetX = bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d().getX();
                double targetY = bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d().getY();
                double targetYaw = bestTarget.getYaw();
                Rotation2d cameraToTargetRotation = Rotation2d.fromDegrees(targetYaw);
                Translation2d cameraToTargetTranslation = new Translation2d(targetX, targetY);
                Transform2d cameraToTarget2D = new Transform2d(cameraToTargetTranslation, cameraToTargetRotation);

                return PhotonUtils.estimateFieldToRobot(cameraToTarget2D, tagPose2D, new Transform2d(
                    robotToCamera.getTranslation().toTranslation2d(), robotToCamera.getRotation().toRotation2d()));
            }
        }
        return new Pose2d(); // Return an empty pose if no valid target
    }

    public double getCameraTimeStampSec() {
        return this.result != null ? this.result.getTimestampSeconds() : 0;
    }

}
