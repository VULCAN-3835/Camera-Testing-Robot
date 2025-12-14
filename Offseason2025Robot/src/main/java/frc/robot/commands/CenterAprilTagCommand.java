package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.Util.LimelightUtil;

public class CenterAprilTagCommand extends Command {
    private final ChassisSubsystem chassisSubsystem;
    private final PIDController txController;
    private final PIDController tyController;

    private double txSetPoint;
    private double tySetPoint;

    private boolean leftCam;
    private LimelightUtil cam;

    private double txSetPointLeft;
    private double txSetPointRight;

    private double tySetPointLeft;
    private double tySetPointRight;

    public CenterAprilTagCommand(ChassisSubsystem chassisSubsystem, boolean leftCam) {
        this.chassisSubsystem = chassisSubsystem;
        this.leftCam = leftCam;
        this.cam = leftCam? this.chassisSubsystem.getLeftCam():this.chassisSubsystem.getRightCam();
        addRequirements(chassisSubsystem);

        this.txSetPointLeft = -3.565;
        this.txSetPointRight = -7.54;

        this.tySetPointLeft = 2.315;
        this.tySetPointRight = 0.04;

        // Tune these gains
        this.txController = new PIDController(0.025, 0, 0);
        this.txController.setTolerance(0.8, 10);
        this.txSetPoint = leftCam ? txSetPointLeft : txSetPointRight; 
        this.txController.setSetpoint(txSetPoint);

        this.tyController = new PIDController(0.04, 0, 0);
        this.tyController.setTolerance(0.15);
        this.tySetPoint = leftCam ? tySetPointLeft : tySetPointRight; 
        this.tyController.setSetpoint(tySetPoint);
    }

    @Override
    public void execute() {
        double xVelocity = tyController.calculate(this.cam.getY());
        xVelocity *= leftCam? -1:1; 

        double yVelocity = -txController.calculate(this.cam.getX());
        yVelocity *= leftCam? -1:1;
        
        SmartDashboard.putNumber("CMD/y velo", yVelocity);
        SmartDashboard.putNumber("CMD/curr", this.cam.getX());
        SmartDashboard.putNumber("CMD/error", txController.getError());
        SmartDashboard.putNumber("CMD/error der", txController.getErrorDerivative());
        SmartDashboard.putNumber("CMD/setpoint", txController.getSetpoint());
        SmartDashboard.putBoolean("CMD/txController.atSetpoint()", txController.atSetpoint());


        SmartDashboard.putNumber("CMD/x velo", xVelocity);
        SmartDashboard.putNumber("CMD/curr Ty", this.cam.getY());
        SmartDashboard.putNumber("CMD/error Ty", tyController.getError());
        SmartDashboard.putNumber("CMD/error der Ty", tyController.getErrorDerivative());
        SmartDashboard.putNumber("CMD/ Ty setpoint", tyController.getSetpoint());
        SmartDashboard.putBoolean("CMD/tyController.atSetpoint()", tyController.atSetpoint());

        this.chassisSubsystem.drive(xVelocity, yVelocity, 0, false);
    }

    @Override
    public boolean isFinished() {
        return 
                (tyController.atSetpoint()&&txController.atSetpoint()) ||
                !this.cam.hasValidTarget();
    }

    @Override
    public void end(boolean interrupted) {
        chassisSubsystem.drive(0, 0, 0, false);
    }
}
