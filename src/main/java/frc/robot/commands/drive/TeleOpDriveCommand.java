package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
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
        double x = driverController.getRawAxis(DriverControllerConsts.LEFT_JOYSTICK_Y),
            y = driverController.getRawAxis(DriverControllerConsts.LEFT_JOYSTICK_X),
            z = driverController.getRawAxis(DriverControllerConsts.RIGHT_JOYSTICK_X);
        x = x < DriverControllerConsts.DEADZONE && x > -1 * DriverControllerConsts.DEADZONE ? 0 : x;
        y = y < DriverControllerConsts.DEADZONE && y > -1 * DriverControllerConsts.DEADZONE ? 0 : y;
        z = z < DriverControllerConsts.DEADZONE && z > -1 * DriverControllerConsts.DEADZONE ? 0 : z;

        if (driverController.getRawAxis(DriverControllerConsts.FINE_CONTROL_AXIS) >= DriverControllerConsts.FINE_CONTOL_THRESHOLD) {
            x *= DriveConstants.FINE_SPEED_REDUCTION;
            y *= DriveConstants.FINE_SPEED_REDUCTION;
            z *= DriveConstants.FINE_SPEED_REDUCTION;
        }

        driveSubsystem.drive(
            x,
            y,
            z,
            driverController.getRawAxis(DriverControllerConsts.HEADED_MODE_AXIS) < DriverControllerConsts.HEADED_MODE_THRESHOLD,
            true
        );

        // debug data
        // TODO: fix the raw axis outputs to match the drive method
        /*final double X = x, Y = y, Z = z;
        SmartDashboard.putNumber("joystick/X", X);
        SmartDashboard.putNumber("joystick/Y", Y);
        SmartDashboard.putNumber("joystick/Z", Z);

        SmartDashboard.putNumber("joystick/rawX", driverController.getRightY());
        SmartDashboard.putNumber("joystick/rawY", driverController.getRightX());
        SmartDashboard.putNumber("joystick/rawZ", driverController.getLeftX());

        SmartDashboard.putBoolean("swerve/running", true);*/
    }

    @Override
    public void end(boolean interrupted) {
        //SmartDashboard.putBoolean("swerve/running", false);
    }
}
