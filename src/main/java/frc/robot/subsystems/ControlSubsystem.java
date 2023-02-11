package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.PeriodicCounter;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LEDColors;
import frc.robot.Constants.IOConstants.CoDriverControllerConsts;
import frc.robot.Constants.IOConstants.DriverControllerConsts;
import frc.robot.Constants.IOConstants.HomingControllerConsts;
import frc.robot.commands.SetLightValueCommand;
import frc.robot.commands.SetWheelLockStateCommand;
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
    private final CommandPS4Controller driverController = new CommandPS4Controller(IOConstants.DRIVER_CONTROLLER_PORT);
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
    }


    //CANSparkMax testMotor = new CANSparkMax(20, MotorType.kBrushless);
    // TODO: convert driving code into command so ControlSubsystem can go byebye
    @Override
    public void periodic() {
        // drive robot
        // TODO: change the fieldRelative field from false to a configurable/constant
        double x = driverController.getRightY(), y = driverController.getRightX(), z = driverController.getLeftX();
        x = x < DriverControllerConsts.DEADZONE && x > -1 * DriverControllerConsts.DEADZONE ? 0 : x;
        y = y < DriverControllerConsts.DEADZONE && y > -1 * DriverControllerConsts.DEADZONE ? 0 : y;
        z = z < DriverControllerConsts.DEADZONE && z > -1 * DriverControllerConsts.DEADZONE ? 0 : z;

        driveSubsystem.drive(x, y, z, false);

        /*if (driverController.button(2).getAsBoolean()) {
            testMotor.set(0.1);
        } else {
            testMotor.set(0.0);
        }*/

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
