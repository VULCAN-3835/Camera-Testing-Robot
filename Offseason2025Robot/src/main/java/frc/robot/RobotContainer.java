// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndAccessoryConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ElevatorConstants.elevatorStates;
import frc.robot.commands.BranchFix;
import frc.robot.commands.CenterAprilTagCommand;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.FaceAprilTagCommand;

import frc.robot.subsystems.ChassisSubsystem;

import frc.robot.subsystems.LEDSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  ChassisSubsystem chassisSubsystem = new ChassisSubsystem();

  LEDSubsystem ledSubsystem = new LEDSubsystem();

  public static elevatorStates targetState;

  private final CommandXboxController xboxControllerDrive = new CommandXboxController(
      OperatorConstants.driverControllerPort);
  private final CommandXboxController buttonXboxController = new CommandXboxController(
      OperatorConstants.buttonControllerPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("EMPTY", null);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    this.targetState = elevatorStates.L2;

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    setUpContollers(false); // make true if using one controller

  }

  private void setUpContollers(boolean oneController) {

    chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(chassisSubsystem,
        () -> xboxControllerDrive.getLeftY(),
        () -> xboxControllerDrive.getLeftX(),
        () -> xboxControllerDrive.getRightX()));

    xboxControllerDrive.start().onTrue(new InstantCommand(() -> chassisSubsystem.zeroHeading()));

    

    
    configureButtonBinding(oneController ? xboxControllerDrive : buttonXboxController);
  }

  private void configureButtonBinding(CommandXboxController cmdXboxController) {
    // Here we will configure the button bindings

    // uses A for L1
    cmdXboxController.a().onTrue(new InstantCommand(() -> targetState = elevatorStates.L1));

    // uses B for L2
    cmdXboxController.b().onTrue(new InstantCommand(() -> targetState = elevatorStates.L2));

    // uses X for L3
    cmdXboxController.x().onTrue(new InstantCommand(() -> targetState = elevatorStates.L3));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
