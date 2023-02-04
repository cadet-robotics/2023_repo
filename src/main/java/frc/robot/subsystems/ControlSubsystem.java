package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.PeriodicCounter;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LEDColors;
import frc.robot.Constants.IOConstants.CoDriverController;
import frc.robot.Constants.IOConstants.DriverController;
import frc.robot.Constants.IOConstants.HomingController;
import frc.robot.commands.SetLightValueCommand;
import frc.robot.commands.controls.codriver.SetWheelLockStateCommand;
import frc.robot.commands.homing.ResetEncodersCommand;
import frc.robot.commands.homing.RotateWheelCommand;
import frc.robot.commands.homing.SetHomingModeCommand;
import frc.robot.commands.homing.ZeroHeadingCommand;
import frc.robot.commands.homing.ZeroSwerveCommand;
import frc.robot.subsystems.DriveSubsystem.SwerveModules;

/**
 * Subsystem for interfacing the robot's functionality with its user inputs
 * @author MattaRama
 */
public class ControlSubsystem extends SubsystemBase {
    // Driver input configuration
    private final CommandJoystick driverStick = new CommandJoystick(IOConstants.DRIVER_CONTROLLER_PORT);
    private final CommandPS4Controller codriverController = new CommandPS4Controller(IOConstants.CODRIVER_CONTROLLER_PORT);
    private final CommandPS4Controller homingController = new CommandPS4Controller(IOConstants.HOMING_CONTROLLER_PORT);

    // Smartdashboard debug caller
    private final PeriodicCounter debugCounter = new PeriodicCounter(15);

    // Required subsystems
    private DriveSubsystem driveSubsystem;
    private LEDSubsystem ledSubsystem;

    public ControlSubsystem(DriveSubsystem driveSubsystem, LEDSubsystem ledSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ledSubsystem = ledSubsystem;

        initBindings();
    }

    public void initBindings() {
        // homing mode
        homingController.button(HomingController.INSERT_HOMING_MODE)
            .onTrue(new SetHomingModeCommand(driveSubsystem, true));
        homingController.button(HomingController.EXIT_HOMING_MODE)
            .onTrue(new SetHomingModeCommand(driveSubsystem, false));

        homingController.button(HomingController.RESET_ENCODERS_BUTTON)
            .onTrue(new ResetEncodersCommand(driveSubsystem));
        homingController.button(HomingController.ZERO_HEADING_BUTTON)
            .onTrue(new ZeroHeadingCommand(driveSubsystem));

        homingController.button(HomingController.ROTATE_FL_BUTTON)
            .whileTrue(new RotateWheelCommand(
                SwerveModules.m_frontLeft.getTurningSparkMax(),
                homingController
            ));
        homingController.button(HomingController.ROTATE_FR_BUTTON)
            .whileTrue(new RotateWheelCommand(
                SwerveModules.m_frontRight.getTurningSparkMax(),
                homingController
            ));
        homingController.button(HomingController.ROTATE_BL_BUTTON)
            .whileTrue(new RotateWheelCommand(
                SwerveModules.m_backLeft.getTurningSparkMax(),
                homingController
            ));
        homingController.button(HomingController.ROTATE_BR_BUTTON)
            .whileTrue(new RotateWheelCommand(
                SwerveModules.m_backRight.getTurningSparkMax(),
                homingController
            ));
        homingController.button(HomingController.ZERO_SWERVE_MODULES)
            .onTrue(new ZeroSwerveCommand(driveSubsystem));

        // wheel lock (X formation)
        codriverController.button(CoDriverController.WHEEL_LOCK_BUTTON)
            .onTrue(new SetWheelLockStateCommand(driveSubsystem, true));
        codriverController.button(CoDriverController.WHEEL_UNLOCK_BUTTON)
            .onTrue(new SetWheelLockStateCommand(driveSubsystem, false));

        // led controls
        codriverController.button(CoDriverController.GREEN_LIGHT)
            .onTrue(new SetLightValueCommand(ledSubsystem, LEDColors.GREEN));
        codriverController.button(CoDriverController.RED_LIGHT)
            .onTrue(new SetLightValueCommand(ledSubsystem, LEDColors.RED));
        codriverController.button(CoDriverController.BLUE_LIGHT)
            .onTrue(new SetLightValueCommand(ledSubsystem, LEDColors.BLUE));
        codriverController.button(CoDriverController.YELLOW_LIGHT)
            .onTrue(new SetLightValueCommand(ledSubsystem, LEDColors.YELLOW));
    }

    @Override
    public void periodic() {
        // drive robot
        // TODO: change the fieldRelative field from false to a configurable/constant
        double x = driverStick.getY(), y = driverStick.getX(), z = driverStick.getZ();
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
            SmartDashboard.putBoolean("homingModeEnabled", driveSubsystem.homingMode);

            // time left in match
            SmartDashboard.putNumber("timeRemaining", Timer.getMatchTime());
        });
    }
}
