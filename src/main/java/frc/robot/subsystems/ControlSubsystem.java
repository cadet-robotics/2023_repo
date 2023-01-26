package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Constants.IOConstants;

/**
 * Subsystem for interfacing the robot's functionality with its user inputs
 * @author MattaRama
 */
public class ControlSubsystem extends SubsystemBase {
    // Driver input configuration
    private final CommandJoystick driverStick = new CommandJoystick(IOConstants.DRIVER_CONTROLLER_PORT);
    private final CommandPS4Controller codriverController = new CommandPS4Controller(IOConstants.CODRIVER_CONTROLLER_PORT);

    // Required subsystems
    private DriveSubsystem driveSubsystem;

    public ControlSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void periodic() {
        // drive robot
        // TODO: change the fieldRelative field from false to a configurable/constant
        driveSubsystem.drive(driverStick.getX(), driverStick.getY(), driverStick.getZ(), false);
    }
}
