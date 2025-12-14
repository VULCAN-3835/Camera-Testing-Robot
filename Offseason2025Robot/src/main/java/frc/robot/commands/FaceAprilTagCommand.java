// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.LimelightUtil;
import frc.robot.subsystems.ChassisSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FaceAprilTagCommand extends Command {
  private final ChassisSubsystem chassisSubsystem;
  private final PIDController yawController;


  
  private double yawSetPoint;
  private boolean leftCam;
  private LimelightUtil cam;


  public FaceAprilTagCommand(ChassisSubsystem chassisSubsystem,boolean leftCam) {
    this.chassisSubsystem = chassisSubsystem;
    this.leftCam = leftCam;
    this.cam = leftCam? this.chassisSubsystem.getLeftCam():this.chassisSubsystem.getRightCam();
    addRequirements(chassisSubsystem);

    
    if (!leftCam) {
      this.yawController = new PIDController(0.2, 0, 0.1);// 0.2 0 0.1

    } else{
      this.yawController = new PIDController(0.15, 0, 0.05);// 0.15 0 0.05
    }
    this.yawController.setTolerance(1.5, 1); // degrees
    this.yawSetPoint = 0; 
    this.yawController.setSetpoint(yawSetPoint);

  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        double rotVelocity = -yawController.calculate(this.cam.getTargetYaw());
        //rotVelocity *= leftCam? -1:1;
        
        SmartDashboard.putNumber("CMD/yaw velo", rotVelocity);
        SmartDashboard.putNumber("CMD/curr yaw", this.cam.getTargetYaw());
        SmartDashboard.putNumber("CMD/ yaw error", yawController.getError());
        SmartDashboard.putNumber("CMD/error der yaw", yawController.getErrorDerivative());
        SmartDashboard.putNumber("CMD/yaw setpoint", yawController.getSetpoint());
        SmartDashboard.putBoolean("CMD/yawController.atSetpoint()", yawController.atSetpoint());
        
        this.chassisSubsystem.drive(0, 0, rotVelocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.drive(0, 0, 0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (yawController.atSetpoint());
    //  || !this.cam.hasValidTarget();
  }
}
