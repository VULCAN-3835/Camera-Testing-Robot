// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor; // Kraken X60 motor responsible for driving the module
    private TalonFX steerMotor; // Falcon 500 motor responsible for steering the module
    private CANcoder absEncoder; // Absolute encoder responsible for keeping track of module position

    private final double absoluteEncoderOffset; // The offset of the absolute encoder from it's true zero

    private SwerveModuleState state; // The state the module aims to be

    // Phoenix6 suppliers for the different feedback values
    private StatusSignal<Angle> m_drivePosition; // Drive position supplier
    private StatusSignal<AngularVelocity> m_driveVelocity; // Drive velocity supplier
    private StatusSignal<Voltage> m_driveVoltage; // Drive voltage supplier
    private StatusSignal<Angle> m_steerPosition; // Steer position supplier
    private StatusSignal<AngularVelocity> m_steerVelocity; // Steer velocity supplier

    private SwerveModulePosition swervePosition = new SwerveModulePosition(); // The position of the module (Releveant for poseEstimator)

    private PositionVoltage angleController = new PositionVoltage(0); // The closed loop controller for module angle
    private VelocityVoltage velocityController = new VelocityVoltage(0);

    private SimpleMotorFeedforward driveFeedForward;

    public SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID, 
    boolean driveMotorInverted,double absoluteEncoderOffset, SimpleMotorFeedforward ff) {
        // Motor controllers + Sensors initialization:
        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new TalonFX(steerMotorID);
        this.absEncoder = new CANcoder(absEncoderID);

        // Offset storing:
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        // Advannced configs:
        configEnc();
        configSteerMotor(absEncoderID);
        configDriveMotor(driveMotorInverted);

        // Storing the signals (suppliers) of the different feedback sensors
        this.m_drivePosition = this.driveMotor.getPosition();
        this.m_driveVelocity = this.driveMotor.getVelocity();
        this.m_driveVoltage = this.driveMotor.getMotorVoltage();
        this.m_steerPosition = this.absEncoder.getPosition();
        this.m_steerVelocity = this.absEncoder.getVelocity();

        // Config for the angle closed loop control feedforward value to overcome friction
        this.angleController.FeedForward = Constants.ModuleConstants.kFeedforwardGainSteer;

        this.driveFeedForward = ff;
    }

    /**
     * Configures encoder offset
    */
    private void configEnc() { 
        CANcoderConfiguration canConfigs = new CANcoderConfiguration();

        canConfigs.MagnetSensor.MagnetOffset = this.absoluteEncoderOffset; // Sets the offset of the Cancoder

        this.absEncoder.getConfigurator().apply(canConfigs);
    }

    /**
     * Configures the steer motor's feedback sensor, source, rotor to sensor ratio and countinous readings
     *
     * @param absEncoderID The id of the absolute encoder to set as the new feedback sensor
    */
    private void configSteerMotor(int absEncoderID) { 
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();

        steerConfigs.Slot0 = Constants.ModuleConstants.getSteerMotorGains(); // Sets constant closed loop values

        steerConfigs.Feedback.FeedbackRemoteSensorID = absEncoderID; // Changes the feedback sensor of the Steer motor into Cancoder
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // Sets the source into can coder
        steerConfigs.Feedback.RotorToSensorRatio = Constants.ModuleConstants.kSteerMotorGearRatio; // Sets a ration between motor and sensor

        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true; // Changes closed loop into continues values

        steerConfigs.CurrentLimits.SupplyCurrentLimit = ModuleConstants.kSteerCurrentLimit;
        steerConfigs.CurrentLimits.SupplyCurrentLowerLimit = ModuleConstants.kSteerCurrentThreshold;
        steerConfigs.CurrentLimits.SupplyCurrentLowerTime = ModuleConstants.kSteerCurrentThresholdTime;
        steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.kSteerEnableCurrentLimit;

        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        this.steerMotor.getConfigurator().apply(steerConfigs);
    }

    /**
     * Configures the drive motor's direction and supply limits and thresholds
     * @param inverted Invert direction of drive motor
    */
    private void configDriveMotor(boolean inverted) {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();

        driveConfigs.Feedback.SensorToMechanismRatio = ModuleConstants.kDriveMotorGearRatio;

        driveConfigs.CurrentLimits.SupplyCurrentLimit = ModuleConstants.kDriveCurrentLimit;
        driveConfigs.CurrentLimits.SupplyCurrentLowerLimit = ModuleConstants.kDriveCurrentThreshold;
        driveConfigs.CurrentLimits.SupplyCurrentLowerTime = ModuleConstants.kDriveCurrentThresholdTime;
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.kDriveEnableCurrentLimit;

        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfigs.MotorOutput.Inverted = inverted?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive;

        this.driveMotor.getConfigurator().apply(driveConfigs);
        this.driveMotor.getConfigurator().setPosition(0);
    }

    /**
     * Returns the module's angle closed loop controller error in rotations (i.e 1 is a full rotation, 0.25 a qurter of a rotation or 90 degrees)
     * @return The module's angle closed loop controller error in rotations
    */
    public double getModuleAngleError() {
        var error = this.steerMotor.getClosedLoopError();
        error.refresh();
        return error.getValue();
    }
    
    /**
     * Returns the module's angle closed loop controller output to the motor in power precentage (i.e between -1 to 1 where 1 is full 
     * power to the positive side of the motor and -1 is full power to the negative side of the motor)
     * @return the module's angle closed loop controller output to the motor
    */
    public double getModuleClosedLoopOutput() {
        var output = this.steerMotor.getClosedLoopOutput();
        output.refresh();
        return output.getValue();
    }

    /**
     * Getter for the current power output the drive motor is recieving
     * @return the drive motor's current power output in precentage (0 - 1)
    */
    public double getModuleDriveOutput() {
        var output = this.driveMotor.getClosedLoopOutput();
        output.refresh();
        return output.getValue();
    }

    /**
     * Returns the module's position using the feedback sensors with latency compensation
     * @return the module's current position
    */
    public SwerveModulePosition getPosition() {
        m_drivePosition.refresh();
        m_driveVelocity.refresh();
        m_steerPosition.refresh();
        m_steerVelocity.refresh();

        Measure<AngleUnit> drive_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);

        Measure<AngleUnit> angle_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        swervePosition.distanceMeters = -drive_rot.in(Rotations) *ModuleConstants.kWheelCircumference ; // Current module's drive position
        swervePosition.angle = Rotation2d.fromDegrees(angle_rot.in(Degrees)); // Current module's angle in Rotation2d

        return swervePosition;
    }

    /**
     * Sets the motor's closed loop controllers setpoints to a certain module state
     * @param state The target state for the module
    */
    public void set(SwerveModuleState state) {
        this.state = state;

        // Refreshes status signals:
        m_steerPosition.refresh();
        m_steerVelocity.refresh();

        // Calculates latency compensated module's angle in rotations
        Measure<AngleUnit> angle_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);


        // Optimizing module's desired state
        state.optimize(Rotation2d.fromDegrees(angle_rot.in(Degrees)));

        // To prevent the wheels from resetting back into angle 0 when not given any input
        if (Math.abs(state.speedMetersPerSecond) < Constants.ModuleConstants.kModuleAngleDeadband) {
            stopModule();
            return;
        }

        // Change feedforward value based on negative / positive error
        this.angleController.FeedForward = getModuleAngleError()>=0?Constants.ModuleConstants.kFeedforwardGainSteer:-Constants.ModuleConstants.kFeedforwardGainSteer;

        // The desired module's angle in rotations
        double angleToSetDeg = state.angle.getRotations();

        this.steerMotor.setControl(this.angleController.withPosition(angleToSetDeg));
        setSpeed(state);
                // this.driveMotor.set(optimized.speedMetersPerSecond);

    }

    /**
     * Sets direct voltage to the drive motor while maintaining wheel angle at base position
     * @param voltage The voltage to apply to the motor
    */
    public void setMotorVoltage(double voltage) {
        this.steerMotor.setControl(this.angleController.withPosition(0));
        this.driveMotor.setVoltage(voltage);
    }

    /**
     * Sets the drive motor's applied voltage control based on a desired state with MPS velocity
     * @param desiredState the desired state for the module with it's speed being in MPS
    */
    private void setSpeed(SwerveModuleState desiredState) {
        this.velocityController.Velocity = desiredState.speedMetersPerSecond* ModuleConstants.kWheelCircumference;
        this.velocityController.FeedForward = this.driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        this.driveMotor.setControl(this.velocityController);
    }

    /**
     * Stops the module's motors
    */
    public void stopModule() {
        this.driveMotor.set(0);
        this.steerMotor.set(0);
    }

    /**
     * Getter for the current velocity (RPM) of the robot
     * @return The velocity of the robot (RPM)
    */
    public double getVelocity() {
        this.m_driveVelocity.refresh();
        return this.m_driveVelocity.getValue().in(RPM)* ModuleConstants.kWheelCircumference;
    }

    /**
     * Getter for the current applied voltage on the drive motor
     * @return The currently applied voltage on the drive motor
    */
    public Voltage getVoltage() {
        this.m_driveVoltage.refresh();
        return m_driveVoltage.getValue();
    }
}
