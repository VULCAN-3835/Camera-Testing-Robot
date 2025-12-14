// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ChassisSubsystem;

public class DefaultTeleopCommand extends Command {
  private final ChassisSubsystem chassisSubsystem;
  private final Supplier<Double> xVelocitySupplier, yVelocitySupplier, turningVelocitySupplier;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private boolean fieldOriented;

    /** Creates a new DefaultTeleopCommand. */
  public DefaultTeleopCommand(ChassisSubsystem chassisSubsystem, Supplier<Double> xVelocitySupplier,
    Supplier<Double> yVelocitySupplier, Supplier<Double> turningVelocitySupplier) {

    this.chassisSubsystem = chassisSubsystem;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.turningVelocitySupplier = turningVelocitySupplier;

    // Initilizing slew rate limiters for acceleration limitations
    this.xLimiter = new SlewRateLimiter(Constants.ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec);
    this.yLimiter = new SlewRateLimiter(Constants.ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec);
    this.turningLimiter = new SlewRateLimiter(Constants.ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec);

    fieldOriented = true;
    
    addRequirements(this.chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Variables to store the gains of input
    double xVelocity = xVelocitySupplier.get();
    double yVelocity = yVelocitySupplier.get();
    double turningVelocity = turningVelocitySupplier.get();

    // Deadband check for the input gains
    xVelocity = Math.abs(xVelocity) > Constants.OperatorConstants.kDeadband ? xVelocity : 0;
    yVelocity = Math.abs(yVelocity) > Constants.OperatorConstants.kDeadband ? yVelocity : 0;
    turningVelocity = Math.abs(turningVelocity) > Constants.OperatorConstants.kDeadband ? turningVelocity : 0;

    // Calculate velocity using gains with constant acceleration and max speed
    xVelocity = xLimiter.calculate(xVelocity) * Constants.ChassisConstants.kTeleDriveMaxSpeedMetersPerSec;
    yVelocity = yLimiter.calculate(yVelocity) * Constants.ChassisConstants.kTeleDriveMaxSpeedMetersPerSec;
    turningVelocity = turningLimiter.calculate(turningVelocity) * Constants.ChassisConstants.kTeleDriveMaxAngulerSpeedRadiansPerSec;

    this.chassisSubsystem.drive(xVelocity, yVelocity, turningVelocity, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
