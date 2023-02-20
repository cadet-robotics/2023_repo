package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IOConstants.DriverControllerConsts;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private DriverController driverController;

    public TeleOpDriveCommand(DriveSubsystem driveSubsystem, DriverController driverController) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
    }

    @Override
    public void execute() {
        // drive robot
        // TODO: change the fieldRelative field from false to a configurable/constant
        double x = driverController.getLeftY(), y = driverController.getLeftX(), z = driverController.getRawAxis(4);
        x = x < DriverControllerConsts.DEADZONE && x > -1 * DriverControllerConsts.DEADZONE ? 0 : x;
        y = y < DriverControllerConsts.DEADZONE && y > -1 * DriverControllerConsts.DEADZONE ? 0 : y;
        z = z < DriverControllerConsts.DEADZONE && z > -1 * DriverControllerConsts.DEADZONE ? 0 : z;

        // speed (TEMPORARY; TODO: remove this for comp, warehouse only code)
        final double DRIVE_SPEED = 0.75;

        x *= DRIVE_SPEED;
        y *= DRIVE_SPEED;
        z *= DRIVE_SPEED;

        driveSubsystem.drive(x, y, z, true);

        // debug data
        // TODO: fix the raw axis outputs to match the drive method
        final double X = x, Y = y, Z = z;
        SmartDashboard.putNumber("joystick/X", X);
        SmartDashboard.putNumber("joystick/Y", Y);
        SmartDashboard.putNumber("joystick/Z", Z);

        SmartDashboard.putNumber("joystick/rawX", driverController.getRightY());
        SmartDashboard.putNumber("joystick/rawY", driverController.getRightX());
        SmartDashboard.putNumber("joystick/rawZ", driverController.getLeftX());

        SmartDashboard.putBoolean("swerve/running", true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("swerve/running", false);
    }
}
