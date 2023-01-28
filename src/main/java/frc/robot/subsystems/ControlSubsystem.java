package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.PeriodicCounter;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IOConstants.CoDriverController;
import frc.robot.Constants.IOConstants.DriverController;
import frc.robot.commands.controls.codriver.SetWheelLockStateCommand;

/**
 * Subsystem for interfacing the robot's functionality with its user inputs
 * @author MattaRama
 */
public class ControlSubsystem extends SubsystemBase {
    // Driver input configuration
    private final CommandJoystick driverStick = new CommandJoystick(IOConstants.DRIVER_CONTROLLER_PORT);
    private final CommandPS4Controller codriverController = new CommandPS4Controller(IOConstants.CODRIVER_CONTROLLER_PORT);
    
    // Smartdashboard debug caller
    private final PeriodicCounter debugCounter = new PeriodicCounter(10);

    // Required subsystems
    private DriveSubsystem driveSubsystem;

    public ControlSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        initBindings();
    }

    public void initBindings() {
        // 

        // wheel lock (X formation)
        codriverController.button(CoDriverController.WHEEL_LOCK_BUTTON)
            .onTrue(new SetWheelLockStateCommand(driveSubsystem, true));
        codriverController.button(CoDriverController.WHEEL_UNLOCK_BUTTON)
            .onTrue(new SetWheelLockStateCommand(driveSubsystem, false));
    }

    @Override
    public void periodic() {
        // drive robot
        // TODO: change the fieldRelative field from false to a configurable/constant
        double x = driverStick.getX(), y = driverStick.getY(), z = driverStick.getZ();
        x = x < DriverController.DEADZONE && x > -1 * DriverController.DEADZONE ? 0 : x;
        y = y < DriverController.DEADZONE && y > -1 * DriverController.DEADZONE ? 0 : y;
        z = z < DriverController.DEADZONE && z > -1 * DriverController.DEADZONE ? 0 : z;
        driveSubsystem.drive(x, y, z, false);

        // debug data
        final double X = x, Y = y, Z = z;
        debugCounter.periodic(() -> {
            SmartDashboard.putNumber("joystick/x", X);
            SmartDashboard.putNumber("joystick/y", Y);
            SmartDashboard.putNumber("joystick/z", Z);
            SmartDashboard.putBoolean("driveEnabled", driveSubsystem.getDriveEnabled());       
        });
    }
}
