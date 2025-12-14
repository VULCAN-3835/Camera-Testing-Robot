package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChassisSubsystem;

public class BranchFix extends Command {
    private final ChassisSubsystem chassis;
    private final boolean goRight;
    private final PIDController turnPID;
    private double targetAngle;

    public BranchFix(ChassisSubsystem chassis, boolean goRight) {
        this.chassis = chassis;
        this.goRight = goRight;
        addRequirements(chassis);

        this.turnPID = new PIDController(0.03, 0, 0); // Adjust P gain as needed
        turnPID.setTolerance(0.6); 
    }

    @Override
    public void initialize() {
        double currentHeading = chassis.getYaw();
        targetAngle = currentHeading + (goRight ? 15.0 : -15.0);
    }

    @Override
    public void execute() {
        double currentHeading = chassis.getYaw();
        double output = turnPID.calculate(currentHeading, targetAngle);

        chassis.drive(0, 0, output, false); 
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return turnPID.atSetpoint();
    }
}
