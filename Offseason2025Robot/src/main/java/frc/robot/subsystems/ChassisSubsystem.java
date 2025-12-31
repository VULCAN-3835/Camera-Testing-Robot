// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.SwerveModule;
import frc.robot.Util.AtCamUtil;
import frc.robot.Util.LimelightUtil;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.time.Instant;
import java.util.List;
import java.util.Optional;

public class ChassisSubsystem extends SubsystemBase {
  // An enum with the names of the wheel modules
  public enum Wheels {
    LEFT_FRONT, RIGHT_FRONT, RIGHT_BACK, LEFT_BACK
  }

  private HolonomicDriveController controller;
  private Optional<Trajectory> trajectory;
  private TrajectoryConfig trajectoryConfig;
  private Timer holonomicTimer;
  private boolean isAutonomous;
  private Pose2d currentPose2dHolonomic;
  private Pose2d holonomicSetPoint;

  private AtCamUtil frontCam;

  // An array of the four swerve Modules
  private SwerveModule[] swerve_modules = new SwerveModule[4];

  // An array of the four swerve module's positions
  private SwerveModulePosition[] swerve_positions = new SwerveModulePosition[4];

  // Inertial Measurement unit
  private AHRS imu;

  // Pose estimator responsible for keeping the robot's position on the field
  // using gyro, encoders and camera detection
  private SwerveDrivePoseEstimator poseEstimator;

  private Pose2d startingPos;

  // Field object for presenting position relative to field
  private Field2d field;
  private Field2d llField;

  // The states of the modules
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(0))
  };

  // Sysid Rotinue
  SysIdRoutine routine;

  public ChassisSubsystem() {
    // Modules Initilization:
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()] = new SwerveModule(
        Constants.ChassisConstants.kLeftFrontDriveID,
        Constants.ChassisConstants.kLeftFrontSteerID,
        Constants.ChassisConstants.kLeftFrontEncID,
        Constants.ChassisConstants.kLeftFrontInverted,
        Constants.ChassisConstants.kLeftFrontOffset,
        ModuleConstants.leftFrontFF);

    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()] = new SwerveModule(
        Constants.ChassisConstants.kRightFrontDriveID,
        Constants.ChassisConstants.kRightFrontSteerID,
        Constants.ChassisConstants.kRightFrontEncID,
        Constants.ChassisConstants.kRightFrontInverted,
        Constants.ChassisConstants.kRightFrontOffset,
        ModuleConstants.rightFrontFF);

    this.swerve_modules[Wheels.LEFT_BACK.ordinal()] = new SwerveModule(
        Constants.ChassisConstants.kLeftBackDriveID,
        Constants.ChassisConstants.kLeftBackSteerID,
        Constants.ChassisConstants.kLeftBackEncID,
        Constants.ChassisConstants.kLeftBackInverted,
        Constants.ChassisConstants.kLeftBackOffset,
        ModuleConstants.leftBackFF);

    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()] = new SwerveModule(
        Constants.ChassisConstants.kRightBackDriveID,
        Constants.ChassisConstants.kRightBackSteerID,
        Constants.ChassisConstants.kRightBackEncID,
        Constants.ChassisConstants.kRightBackInverted,
        Constants.ChassisConstants.kRightBackOffset,
        ModuleConstants.rightBackFF);

    // Imu initlization
    this.imu = new AHRS(NavXComType.kMXP_SPI);

    // Robot starting position for odometry
    startingPos = new Pose2d(1.3, 5.52, Rotation2d.fromDegrees(0));

    // Field initlization
    field = new Field2d();

    llField = new Field2d();
    SmartDashboard.putData("ll field", llField);

    this.isAutonomous = false;

    // Update swerve position and heading at build
    updateSwervePositions();
    zeroHeading();

    this.frontCam = new AtCamUtil("camera 1", new Transform3d(0, 0.3, 0.425, new Rotation3d(0, 0, 0)),this.getRotation2d());


    // Initilizing a pose estimator
    this.poseEstimator = new SwerveDrivePoseEstimator(ChassisConstants.kDriveKinematics,
        getRotation2d().unaryMinus(),
        this.swerve_positions,
        startingPos);

    // Configuring the controller for the path planner
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        () -> ChassisConstants.kDriveKinematics.toChassisSpeeds(getModStates()),
        this::runVelc,
        new PPHolonomicDriveController(
            new PIDConstants(0.3, 0, 0), // Translation PID
            new PIDConstants(0.5, 0, 0) // Rotation PID
        ),
        ChassisConstants.getConfig(),
        () -> !(Robot.allianceColor == "BLUE"),
        this);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData(field);

    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::setSysidVolt,
            log -> {
              // set states for all 4 modules
              for (Wheels wheel : Wheels.values()) {
                log.motor(wheel.toString())
                    .voltage(swerve_modules[wheel.ordinal()].getVoltage())
                    .linearPosition(
                        Distance.ofBaseUnits(swerve_modules[wheel.ordinal()].getPosition().distanceMeters, Meters))
                    .linearVelocity(
                        LinearVelocity.ofBaseUnits(swerve_modules[wheel.ordinal()].getVelocity(), MetersPerSecond));
              }
            }, this));

    SmartDashboard.putData("Desired swerve state", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle",
            () -> swerveModuleStates[Wheels.LEFT_FRONT.ordinal()].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Right Angle",
            () -> swerveModuleStates[Wheels.RIGHT_FRONT.ordinal()].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Left Angle",
            () -> swerveModuleStates[Wheels.LEFT_BACK.ordinal()].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Right Angle",
            () -> swerveModuleStates[Wheels.RIGHT_BACK.ordinal()].angle.getDegrees(), null);

        builder.addDoubleProperty("Front Left Speed",
            () -> swerveModuleStates[Wheels.LEFT_FRONT.ordinal()].speedMetersPerSecond, null);
        builder.addDoubleProperty("Front Right Speed",
            () -> swerveModuleStates[Wheels.RIGHT_FRONT.ordinal()].speedMetersPerSecond, null);
        builder.addDoubleProperty("Back Left Speed",
            () -> swerveModuleStates[Wheels.LEFT_BACK.ordinal()].speedMetersPerSecond, null);
        builder.addDoubleProperty("Back Right Speed",
            () -> swerveModuleStates[Wheels.RIGHT_BACK.ordinal()].speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getYaw(), null);
      }
    });
    SmartDashboard.putData("Current swerve state", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle",
            () -> swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().angle.getDegrees(), null);
        builder.addDoubleProperty("Front Right Angle",
            () -> swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().angle.getDegrees(), null);
        builder.addDoubleProperty("Back Left Angle",
            () -> swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().angle.getDegrees(), null);
        builder.addDoubleProperty("Back Right Angle",
            () -> swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().angle.getDegrees(), null);

        builder.addDoubleProperty("Front Left Speed",
            () -> swerve_modules[Wheels.LEFT_FRONT.ordinal()].getVelocity(), null);
        builder.addDoubleProperty("Front Right Speed",
            () -> swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getVelocity(), null);
        builder.addDoubleProperty("Back Left Speed",
            () -> swerve_modules[Wheels.LEFT_BACK.ordinal()].getVelocity(), null);
        builder.addDoubleProperty("Back Right Speed",
            () -> swerve_modules[Wheels.RIGHT_BACK.ordinal()].getVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getYaw(), null);
      }
    });
  }

  /**
   * Updates the swerve_positions array based on the current SwerveModulePositions
   * reported by the SwerveModules
   */
  private void updateSwervePositions() {
    this.swerve_positions[Wheels.LEFT_FRONT.ordinal()] = this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition();
    this.swerve_positions[Wheels.RIGHT_FRONT.ordinal()] = this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()]
        .getPosition();
    this.swerve_positions[Wheels.LEFT_BACK.ordinal()] = this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition();
    this.swerve_positions[Wheels.RIGHT_BACK.ordinal()] = this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition();
  }

  /**
   * Resets the heading of the robot
   */
  public void zeroHeading() {
    this.imu.reset();
  }
  public AtCamUtil getFrontCam() {
    return this.frontCam;
  }

  /**
   * Returns the heading of the robot
   * 
   * @return The heading of the robot in degrees
   */
  public double getYaw() {
    return -this.imu.getAngle();
  }

  /**
   * Returns the heading of the robot
   * 
   * @return The heading of the robot in Rotation2d
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  public AHRS getGyro() {
    return imu;
  }

  /**
   * Sets the module's state to given one
   *
   * @param xVelocity     The velocity on the x axis
   * @param yVelocity     The velocity on the y axis
   * @param rot           The rotational velocity
   * @param fieldRelative Is field relative or not
   */
  public void drive(double xVelocity, double yVelocity, double rot, boolean fieldRelative) {

    SmartDashboard.putNumber("Target X Velocity", xVelocity);
    SmartDashboard.putNumber("Target Y Velocity", yVelocity);
    SmartDashboard.putNumber("Target rot Velocity", rot);

    boolean invert = false;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red && fieldRelative)
      invert = true;
    // Kinematics turns the Chassis speeds to desired swerveModule states depending
    // on if field relative or not
    this.swerveModuleStates = Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rot,
                invert ? getRotation2d().plus(Rotation2d.k180deg) : getRotation2d())
            : new ChassisSpeeds(xVelocity, yVelocity, rot));

  }

  /**
   * Sets the module's state to given one
   *
   * @param chassisSpeeds The desired chassisSpeeds object for module velocities
   * @param fieldRelative Is field relative or not
   */
  public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
    // Makes a swerve module-state array from chassisSpeeds
    this.swerveModuleStates = Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.imu.getRotation2d())
            : chassisSpeeds);
  }
  // public InstantCommand drive(){
  // return new InstantCommand(()->drive(0,0,0,false),this);
  // }

  /**
   * Runs the robot following trajectory
   *
   * @param speeds The desired chassisSpeeds object for module velocities
   */
  public void runVelc(ChassisSpeeds speeds) {
    ChassisSpeeds discSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(ChassisSpeeds.discretize(speeds, 0.02),
        getRotation2d());
    // ChassisSpeeds discSpeeds = ChassisSpeeds.discretize(speeds, 0.01);

    this.swerveModuleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(discSpeeds);
  }

  /**
   * Sets the desired states of the modules to given ones
   *
   * @param desiredStates Desired array of 4 module states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Sets max acceleration and velocity to Wheels
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.ChassisConstants.kMaxDrivingVelocity);

    // Uses the set method of the SwerveModule to declare desired state of the
    // module
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].set(desiredStates[0]);
    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].set(desiredStates[1]);
    this.swerve_modules[Wheels.LEFT_BACK.ordinal()].set(desiredStates[2]);
    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].set(desiredStates[3]);
  }

  /**
   * Command to execute quasistatic routine based on direction
   * 
   * @param direction The direction of the robot during routine
   * @return The command that executes the routine
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.routine.quasistatic(direction);
  }

  /**
   * Command to execute Dynamic routine based on direction
   * 
   * @param direction The direction of the robot during routine
   * @return The command that executes the routine
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.routine.dynamic(direction);
  }

  /**
   * Setter for swerve modules voltage, used only in sysid routine
   * 
   * @param volts The voltage to apply
   */
  private void setSysidVolt(Voltage volts) {
    double voltageDouble = volts.magnitude();
    setModulesVoltage(voltageDouble);
  }

  /**
   * Setter for swerve module voltage
   * 
   * @param volt The voltage to apply
   */
  public void setModulesVoltage(double volt) {
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.LEFT_BACK.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].setMotorVoltage(volt);
  }

  /**
   * Resets the odometry to a given pose
   * 
   * @param pose The new pose2d of the robot
   */
  public void resetOdometry(Pose2d pose) {
    System.out.println("resets");
    this.poseEstimator.resetPose(pose);
    // this.poseEstimator.resetPosition(getRotation2d().unaryMinus(),
    // getModPositions(), pose);
  }

  /**
   * Getter for the current pose2d estimated by the PoseEstimator
   * 
   * @return The current reported position of the robot
   */
  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  public PoseEstimator getPoseEstimator() {
    return this.poseEstimator;
  }

  /**
   * Getter for the current reported swerve module positions
   * 
   * @return The current reported swerve module positions
   */
  public SwerveModulePosition[] getModPositions() {
    return this.swerve_positions;
  }

  /**
   * Getter for the current reported swerve module states
   * 
   * @return The current reported swerve module states
   */
  public SwerveModuleState[] getModStates() {
    return this.swerveModuleStates;
  }

  /**
   * Update pose estimator using vision data from the At Cam
   */
  private void updatePoseEstimatorWithVisionBotPose() {
    Pose2d visionBotPose = this.frontCam.getPoseFromCamera();
    double xyStds;
    double degStds;

    if (visionBotPose.getX() != 0.0 && this.frontCam.hasValidTarget()) {
      xyStds = 0.5 * (1 / Math.pow(frontCam.distanceFromTargetMeters(), 2));
      degStds = 6;

      poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      poseEstimator.addVisionMeasurement(visionBotPose,
          Timer.getFPGATimestamp() - (frontCam.getCameraTimeStampSec()));
    }

    // if has 2 cams - this one is for limelight
    // if (visionBotPoseSource.getX() != 0.0 &&
    // this.limelightRight.hasValidTarget()) {
    // xyStds = 0.5 * (1 / Math.pow(limelightRight.distanceFromTargetMeters(), 2));
    // degStds = 6;

    // poseEstimator.setVisionMeasurementStdDevs(
    // VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
    // poseEstimator.addVisionMeasurement(visionBotPoseSource,
    // Timer.getFPGATimestamp() - (this.limelightRight.getCameraTimeStampSec()));
  }

  private void initHolonomicDriver() {
    this.controller = new HolonomicDriveController(
        new PIDController(1.75, 0, 0),
        new PIDController(1.75, 0, 0),
        new ProfiledPIDController(5, 0, 0,
            new TrapezoidProfile.Constraints(2, 2)));

    this.trajectoryConfig = new TrajectoryConfig(2, 2);
    this.trajectory = Optional.of(new Trajectory());
    this.holonomicTimer = new Timer();
  }

  public void driveTo(Pose2d pose2d) {
    initHolonomicDriver();
    this.holonomicTimer.reset();
    this.holonomicTimer.start();
    this.isAutonomous = true;
    currentPose2dHolonomic = new Pose2d(this.getPose().getTranslation(),
        pose2d.getTranslation().minus(getPose().getTranslation()).getAngle());
    holonomicSetPoint = new Pose2d(pose2d.getTranslation(),
        pose2d.getTranslation().minus(getPose().getTranslation()).getAngle());

    trajectory = Optional.ofNullable(TrajectoryGenerator.generateTrajectory(
        List.of(currentPose2dHolonomic, holonomicSetPoint), trajectoryConfig));
  }

  public InstantCommand driveToPose2d(Pose2d pose2d) {
    return new InstantCommand(() -> driveTo(pose2d));
  }

  // Chassis SysID to use this paste it in the configureXboxBinding method in
  // robotContainer
  // xboxControllerDrive.a().whileTrue(chassisSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
  // xboxControllerDrive.b().whileTrue(chassisSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  // xboxControllerDrive.y().whileTrue(chassisSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
  // xboxControllerDrive.x().whileTrue(chassisSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

  @Override
  public void periodic() {

    if (isAutonomous) {

      State goalState = trajectory.get().sample(holonomicTimer.get());

      ChassisSpeeds speeds = controller.calculate(getPose(), goalState.poseMeters, goalState.velocityMetersPerSecond,
          holonomicSetPoint.getRotation());
      drive(speeds, false);
      if (Meters.of(currentPose2dHolonomic.minus(holonomicSetPoint).getTranslation()
          .getDistance(holonomicSetPoint.getTranslation())).gt(Centimeters.of(0.5))) {
        isAutonomous = false;
      }
    }

    setModuleStates(this.swerveModuleStates);

    updateSwervePositions();
    this.poseEstimator.update(getRotation2d(), getModPositions());
    updatePoseEstimatorWithVisionBotPose();

    if (this.frontCam.hasValidTarget()) {
      this.poseEstimator.addVisionMeasurement(this.frontCam.getPoseFromCamera(),
      Timer.getFPGATimestamp() - (this.frontCam.getCameraTimeStampSec()));
    }
    // this.field.setRobotPose(this.poseEstimator.getEstimatedPosition());
    this.field.setRobotPose(this.frontCam.getPoseFromCamera());
    frontCam.updateResult();
    SmartDashboard.putBoolean(frontCam.getName()+"/connected", frontCam.isConnected());
    SmartDashboard.putBoolean(frontCam.getName()+"/has valid target", frontCam.hasValidTarget());
    SmartDashboard.putNumber(frontCam.getName()+"/tag ID", frontCam.getID());
    SmartDashboard.putNumber(frontCam.getName()+"/distance from target", frontCam.getDistanceToTarget());


    SmartDashboard.putNumber("ChassisSubsystem/Gyro Yaw", getYaw());

    SmartDashboard.putNumber("ChassisSubsystem/Left Front Distance",
        this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("ChassisSubsystem/Left Back Distance",
        this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("ChassisSubsystem/Right Front Distance",
        this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("ChassisSubsystem/Right Back Distance",
        this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().distanceMeters);

    SmartDashboard.putNumber("ChassisSubsystem/Left Front Rotation",
        this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("ChassisSubsystem/Left Back Rotation",
        this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("ChassisSubsystem/Right Front Rotation",
        this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("ChassisSubsystem/Right Back Rotation",
        this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().angle.getRotations());

    SmartDashboard.putNumber("ChassisSubsystem/Left Front Rotation Error",
        this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("ChassisSubsystem/Left Back Rotation Error",
        this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("ChassisSubsystem/Right Front Rotation Error",
        this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("ChassisSubsystem/Right Back Rotation Error",
        this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleAngleError());

    SmartDashboard.putNumber("ChassisSubsystem/Left Front Rotation Output",
        this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("ChassisSubsystem/Left Back Rotation Output",
        this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("ChassisSubsystem/Right Front Rotation Output",
        this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("ChassisSubsystem/Right Back Rotation Output",
        this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleClosedLoopOutput());

    SmartDashboard.putNumber("ChassisSubsystem/Left Front Drive Velocity",
        swerve_modules[Wheels.LEFT_FRONT.ordinal()].getVelocity());
    SmartDashboard.putNumber("ChassisSubsystem/Left Back Drive Velocity",
        swerve_modules[Wheels.LEFT_BACK.ordinal()].getVelocity());
    SmartDashboard.putNumber("ChassisSubsystem/Right Front Drive Velocity",
        swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getVelocity());
    SmartDashboard.putNumber("ChassisSubsystem/Right Back Drive Velocity",
        swerve_modules[Wheels.RIGHT_BACK.ordinal()].getVelocity());

    SmartDashboard.putNumber("ChassisSubsystem/Left Front Drive Output",
        swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("ChassisSubsystem/Left Back Drive Output",
        swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("ChassisSubsystem/Right Front Drive Output",
        swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("ChassisSubsystem/Right Back Drive Output",
        swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleDriveOutput());

    SmartDashboard.putNumber("match time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("battary voltage", RobotController.getBatteryVoltage());

  }
}
