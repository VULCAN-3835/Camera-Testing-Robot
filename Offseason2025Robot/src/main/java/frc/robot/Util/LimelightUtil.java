// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightUtil {
    private NetworkTable limelight; // Network object that represents the limelight connection

    /** Creates a new LimelightUtil */
    public LimelightUtil(String limelightName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    /**
     * Getter for the x offset from the center of the camera to the target
     * @return The x offset from the center of the camera to the target
    */
    public double getX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    /**
     * Getter for the y offset from the center of the camera to the target
     * @return The y offset from the center of the camera to the target
    */
    public double getY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    /**
     * Getter for the ammount of area the target takes on the cameras screen
     * @return The ammount of area the target takes on the cameras screen
    */
    public double getA() {
        return limelight.getEntry("ta").getDouble(0);
    }

    /**
     * Checks if camera has a valid target
     * @return True if camera has a valid target
    */
    public boolean cameraHasTarget() {
        return this.limelight.getEntry("tv").getDouble(0) != 0;
    }

    /**
     * Changes the camera to april tag detection pipeline
    */
    public void setAprilMode() {
        limelight.getEntry("pipeline").setNumber(0);
    }

    /**
     * Changes the camera to reflection detection pipeline
    */
    public void setReflectorMode() {
        limelight.getEntry("pipeline").setNumber(1);
    }

    /**
     * Getter for the Pose2d object that represents the position of the robot on the field reported by the limelight relative to the blue alliance corner
     * @return A pose2d object representing the position of the robot
    */
    public Pose2d getPoseFromCamera() {
        double[] botpose = this.limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    
        // Defensive check
        if (botpose.length < 6) {
            // Not enough data, return a safe default
            return new Pose2d();
        }
    
        double x = botpose[0];
        double y = botpose[1];
        double yaw = botpose[5];
    
        return new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
    }
    

    /**
     * Getter for the total latency of limelight camera
     * @return The total latency of limelight camera
    */
    public double getCameraTimeStampSec() {
        double timestamp = this.limelight.getEntry("cl").getDouble(0)+this.limelight.getEntry("tl").getDouble(0);
        return timestamp/1000;
    }
    public double getTargetYaw() {
        double[] pose = this.limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        // pose[4] is the yaw in degrees relative to the camera
        return pose[4];
    }
    

    /**
     * Getter for the distance in meters from target
     * @return The distance in meters from target
    */
    public double distanceFromTargetMeters() {
        double[] botpose_apriltag = this.limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double z = botpose_apriltag[2];
        double y = botpose_apriltag[1];
        double x = botpose_apriltag[0];

        return Math.sqrt(Math.pow(z,2)+Math.pow(x,2)+Math.pow(y, 2));
    }

    /**
     * Checks if camera has a target and its closer than 5 meters
     * @return True if target is valid
    */
    public boolean hasValidTarget() {
        return cameraHasTarget() && distanceFromTargetMeters() < 3;
    }

    /**
     * Getter for the current april tag id of the target
     * @return The id number of the target april tag
    */
    public double getAprilTagID() {
        return this.limelight.getEntry("tid").getDouble(0);
    }
}