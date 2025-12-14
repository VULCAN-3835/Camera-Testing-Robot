// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Per;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Rotation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int buttonControllerPort = 1;
    public static final double kDeadband = 0.1;

  }
  public static final class EndAccessoryConstants {
    public static final int leftMotorID = 60; //CAN ID
    public static final int rightMotorID = 61; //CAN ID

    public static final int infraredID = 2; // DIO ID

    public static final double intakePower = 0.12;
    public static final double L1Power = 0.15;
    public static final double L2Power = 0.2;
    public static final double L3Power = 0.2;
    public static final double L4Power = 0.15;
  
  }

  public static class ElevatorConstants{

    public static enum elevatorStates{REST,SOURCE,L1,L2,L3,L4}


    public static final int ElevatorMotorRightID = 50; // CAN ID
    public static final int ElevatorMotorLeftID = 51; // CAN ID
    public static final int limitSwitchID = 0; // DIO ID

    public static final double maxVelc = 20;
    public static final double maxAcc = 40;

    public static final double Kp = 0.25;
    public static final double pidTolerance = 0.15;


    //distance in cm for each level
    public static final double RestHeight = 0;
    //hright was 0.7
    public static final double SourceHeight = 0.7;
    
    public static double L1Height = 4;
    public static double L2Height = 10;
    public static double L3Height = 17.8; 
    public static double L4Height = 27.7;



    // Conversion factor from encoder rotations to cm
    public static Per<DistanceUnit, AngleUnit> distancePerRotation = Centimeters.of(1).div(Rotation.of(3.2));

  }


  
public static class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // Module wheel diameter in meters
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kDriveMotorGearRatio = 6.75; // Module drive motor gear ratio
    public static final double kSteerMotorGearRatio = 12.8; // Module steer motor gear ratio

    public static double kFeedforwardGainSteer = 0.11; // The feed forward gain for the module steer control

    public static Slot0Configs getSteerMotorGains() {
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(30); // The proportional gain for the module steer control
      return kSteerMotorGains;
    }

    public static Slot0Configs getDriveMotorGains() {
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(0.15); // The proportional gain for the module steer control
      return kSteerMotorGains;
    }

    public static SimpleMotorFeedforward leftFrontFF = new SimpleMotorFeedforward(0.21599, 2.2476, 0.040257);
    public static SimpleMotorFeedforward leftBackFF = new SimpleMotorFeedforward(0.20676, 2.1653, 0.040257); // previous
                                                                                                            // constants
                                                                                                            // -
                                                                                                            // (0.20676,
                                                                                                            // 2.1653,
                                                                                                            // 0.16537)
    public static SimpleMotorFeedforward rightFrontFF = new SimpleMotorFeedforward(0.1788, 2.257, 0.040257);
    public static SimpleMotorFeedforward rightBackFF = new SimpleMotorFeedforward(0.11961, 2.3274, 0.040257);

    public static double kModuleAngleDeadband = 0.001;

    /* Swerve Current Limiting */
    public static final int kSteerCurrentLimit = 25;
    public static final int kSteerCurrentThreshold = 30;
    public static final double kSteerCurrentThresholdTime = 0.1;
    public static final boolean kSteerEnableCurrentLimit = true;

    public static final int kDriveCurrentLimit = 30;
    public static final int kDriveCurrentThreshold = 40;
    public static final double kDriveCurrentThresholdTime = 0.1;
    public static final boolean kDriveEnableCurrentLimit = true;
  }
  
  public static class ChassisConstants {

    public static final String reefCam = "limelight-reef";

    // Ports for driving motors
    public static final int kLeftFrontDriveID = 13; // CAN ID
    public static final int kRightFrontDriveID = 12; // CAN ID
    public static final int kLeftBackDriveID = 11; // CAN ID
    public static final int kRightBackDriveID = 10; // CAN ID
    // Ports for angle motors
    public static final int kLeftFrontSteerID = 23; // CAN ID
    public static final int kRightFrontSteerID = 22; // CAN ID
    public static final int kLeftBackSteerID = 21; // CAN ID
    public static final int kRightBackSteerID = 20; // CAN ID
    // Ports for encoders
    public static final int kLeftFrontEncID = 33; // CAN ID
    public static final int kRightFrontEncID = 32; // CAN ID
    public static final int kLeftBackEncID = 31; // CAN ID
    public static final int kRightBackEncID = 30; // CAN ID

    // Offsets for absolute encoders in rotations (i.e: 360 degrees = 1 rotation):
    // public static final double kLeftFrontOffset = -0.029296875;
    // public static final double kRightFrontOffset = -0.4111328125;
    // public static final double kLeftBackOffset = 0.406494140625;
    // public static final double kRightBackOffset = -0.228515625;

    public static final double kLeftFrontOffset = -0.419189;//0.3134765625;
    public static final double kRightFrontOffset = 0.5344;//-0.2197265625;
    public static final double kLeftBackOffset =  0.341 ;//-0.40771484375;
    public static final double kRightBackOffset = -0.014 ;// -0.23779296875;
    // Which motors are inverted: public static final boolean frontLeftDriveInverted
    // = true;
    public static final boolean kLeftFrontInverted = true;
    public static final boolean kRightFrontInverted = true;
    public static final boolean kLeftBackInverted = true;
    public static final boolean kRightBackInverted = true;

    public static final double kMaxDrivingVelocity = 4.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSec = 9;
    public static final double kTeleDriveMaxSpeedMetersPerSec = 4.5;
    public static final double kTeleDriveMaxAngulerSpeedRadiansPerSec = Math.PI * 1.8;

    // Distance between centers of right and left wheels on robot meters
    public static final double kTrackWidth = 0.6357;
    // Distance between front and back wheels on robot meters
    public static final double kWheelBase = 0.6357;
    // Distance between middle of robot to module wheel
    public static final double kWheelRadius = 0.38205;

    // the mass of the robot in KG
    public static final double kMassKG = 51;
    // the moment of inertia of the robot
    public static final double kMOI = 6.81;


    // Swerve Kinematics:
    // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    //     new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // Left front
    //     new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Right front
    //     new Translation2d(kWheelBase / 2,- kTrackWidth / 2), // Left back
    //     new Translation2d(-kWheelBase / 2, kTrackWidth / 2) // Right back
    // );

  //   public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
  //     new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Left front
  //     new Translation2d(kWheelBase / 2,kTrackWidth / 2), // Right front
  //     new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // Left back
  //     new Translation2d(kWheelBase / 2, -kTrackWidth / 2) // Right back
  // );

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Left front
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // Left back
      new Translation2d(kWheelBase / 2,kTrackWidth / 2), // Right front
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2) // Right back
    );


  // );
    public static final RobotConfig DEFAUL_ROBOT_CONFIG = new RobotConfig(kMassKG, kMOI,
        new ModuleConfig(kWheelRadius,
            kMaxDrivingVelocity,
            kWheelBase, DCMotor.getKrakenX60(4),
            ModuleConstants.kDriveCurrentLimit, 1),
        kDriveKinematics.getModules());

    public static RobotConfig getConfig() {
      RobotConfig config;
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();
        config = DEFAUL_ROBOT_CONFIG;
      }
      return config;

    }
  }
}
