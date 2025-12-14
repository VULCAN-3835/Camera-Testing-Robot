package frc.robot.Util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import java.util.Optional;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
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

    private Pose3d cameraPose;

    private PhotonPipelineResult result;
    private List<PhotonPipelineResult> resultLog;

    public AtCamUtil(String nameOfCamera, Pose3d cameraPose){
        //constructing the camera
        this.OVCamera = new PhotonCamera(nameOfCamera);
        updateResult();
        this.OVCamera.setPipelineIndex(1);
        this.cameraPose = cameraPose;
        boolean fieldWork;
        try{
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
    

    public boolean hasTarget() {
        return this.result != null && this.result.hasTargets();
    }

    public void updateResult() {
        PhotonPipelineResult newResult = this.OVCamera.getLatestResult();

        if (newResult != null && newResult.hasTargets()) {
            this.result = newResult;
        }
    }
    public Optional<Pose3d> getTagPose3d(int ID){
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(ID);
        return tagPose;
    }

    public int getID() {
        if (hasTarget()) {
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
    

    public double getDistance() {
        if (!hasTarget()) {
            return -1; // No target detected
        }
    
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(getID());
        if (tagPose.isEmpty()) {
            return -1; // No AprilTag pose available for the given ID
        }
    
        // Extract relevant data safely
        double cameraZ = cameraPose.getTranslation().getZ();
        double targetZ = tagPose.get().getTranslation().getZ();
        double cameraPitch = cameraPose.getRotation().getY();
        double targetPitch = tagPose.get().getRotation().getY();
    
        // Calculate distance
        return PhotonUtils.calculateDistanceToTargetMeters(cameraZ, targetZ, cameraPitch, targetPitch);
    }
    public double distanceFromTargetMeters(){
        if (hasTarget()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                this.cameraPose.getMeasureZ().in(Meters),
                 getTagPose3d(getID()).get().getMeasureZ().in(Meters),
                  this.cameraPose.getRotation().getMeasureY().in(Radians),
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
                    new Translation2d(tagPose3D.getTranslation().toTranslation2d().getX(), tagPose3D.getTranslation().toTranslation2d().getY()),
                    new Rotation2d(tagPose3D.getRotation().getZ())
                ));

            if (tagPoseOptional.isPresent()) {
                Pose2d tagPose2D = tagPoseOptional.get();

                // Manually extract 2D transform values from the target
                double targetX = bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d().getX();
                double targetY = bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d().getY();
                double targetYaw = bestTarget.getYaw(); 
                Rotation2d cameraToTargetRotation = Rotation2d.fromDegrees(targetYaw);
                Translation2d cameraToTargetTranslation = new Translation2d(targetX, targetY);
                Transform2d cameraToTarget2D = new Transform2d(cameraToTargetTranslation, cameraToTargetRotation);

                return PhotonUtils.estimateFieldToRobot(cameraToTarget2D, tagPose2D, new Transform2d(cameraPose.getTranslation().toTranslation2d(), cameraPose.getRotation().toRotation2d()));
            }
        }
        return new Pose2d(); // Return an empty pose if no valid target
    }


    public double getCameraTimeStampSec() {
        return this.result != null ? this.result.getTimestampSeconds() : 0;
    }

}
