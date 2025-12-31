package frc.robot.Util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.net.NetworkInterface;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.common.dataflow.structures.Packet;
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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AtCamUtil {
    private PhotonCamera cam;
    private AprilTagFieldLayout aprilTagFieldLayout;

    private Transform3d robotToCamera;
    private Rotation2d rot;
    private PhotonPipelineResult result;
    private String name;


    public AtCamUtil(String name, Transform3d robotToCamera,Rotation2d rot) {
        // constructing the camera
        this.name = name;
        this.cam = new PhotonCamera(name);
        this.rot = rot;
        updateResult();
        this.cam.setPipelineIndex(1);
        this.robotToCamera = robotToCamera;

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
        return this.cam.isConnected();
    }

    public String getName() {
        return this.name;
    }

    private boolean hasTarget() {
        return this.result != null && this.result.hasTargets();
    }

    public boolean hasValidTarget() {
        // return hasTarget() && getDistanceToTarget() < 1.5;
        return hasTarget();
    }

    // public void updateResult() {
    // List<PhotonPipelineResult> unreadResults = this.cam.getAllUnreadResults();

    // if (unreadResults.size() != 0) {
    // PhotonPipelineResult newResult = unreadResults.get(0);
    // if (newResult != null && newResult.hasTargets()) {
    // this.result = newResult;
    // }
    // }
    // }

    public void updateResult() {
        List<PhotonPipelineResult> unreadResults = this.cam.getAllUnreadResults();

        if (!unreadResults.isEmpty()) {
            PhotonPipelineResult newResult = unreadResults.get(unreadResults.size() - 1);
            if (newResult != null && newResult.hasTargets()) {
                this.result = newResult;
                SmartDashboard.putString(name + "target info", newResult.toString());
            }
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
        if (!hasTarget())
            return -1.0;

        PhotonTrackedTarget target = result.getBestTarget();
        Optional<Pose3d> tagOnField = aprilTagFieldLayout.getTagPose(target.getFiducialId());

        if (tagOnField.isPresent()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    this.robotToCamera.getZ(), // גובה המצלמה
                    tagOnField.get().getZ(), // גובה התג
                    this.robotToCamera.getRotation().getY(), // זווית המצלמה (Pitch)
                    Math.toRadians(target.getPitch()) // זווית המטרה בתמונה
            );
        }
        return -1.0;
    }

    public double distanceFromTargetMeters() {
        if (hasValidTarget()){
            PhotonUtils.calculateDistanceToTargetMeters(this.robotToCamera.getY(),
             this.aprilTagFieldLayout.getTagPose(getID()).get(), getDistanceToTarget(), getCameraTimeStampSec())
        }
    }

    public Pose2d getPoseFromCamera() {
        if (hasValidTarget() ) {
            PhotonTrackedTarget bestTarget = this.result.getBestTarget();

            Optional<Pose3d> tagPose3D = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());

            Transform3d cameraToTarget3d = bestTarget.getBestCameraToTarget();
            Translation3d translation3d = cameraToTarget3d.getTranslation();
            Translation2d cameraToTargetTranslation = translation3d.toTranslation2d();

            Transform2d CamToTarget = PhotonUtils.estimateCameraToTarget(cameraToTargetTranslation, tagPose3D.get().toPose2d(), this.rot);

                return PhotonUtils.estimateFieldToRobot(CamToTarget, tagPose3D.get().toPose2d(),
                        new Transform2d(this.robotToCamera.getX(), this.robotToCamera.getY(),
                                this.robotToCamera.getRotation().toRotation2d()));
            
        }
        return new Pose2d(); // Return an empty pose if no valid target
    }

    public double getCameraTimeStampSec() {
        return this.result != null ? this.result.getTimestampSeconds() : 0;
    }
}
