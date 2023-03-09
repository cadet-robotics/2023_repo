// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AnalogConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.PWMConstants;
import frc.robot.controllers.CoDriverController;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.HomingController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utils.DistanceSensor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{  
    // Control declarations
    public final DriverController driverController;
    public final CoDriverController codriverController;
    public final HomingController homingController;
    
    public final ArmSubsystem armSubsystem;
    public final LEDSubsystem ledSubsystem;
    public final DriveSubsystem driveSubsystem;
    public final ClawSubsystem clawSubsystem;

    // Sensors/extra declarations
    public final DistanceSensor distanceSensor;

    // Auto chooser
    private final SendableChooser<String> autoSelector = new SendableChooser<>();
    private static final String defaultAutoPath = "none";

    // TODO: convert each type of controller into its own wrapper class, and get rid of this subsystem
    //private final ControlSubsystem controlSubsystem = new ControlSubsystem(driveSubsystem, ledSubsystem);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // init controllers
        driverController = new DriverController(IOConstants.DRIVER_CONTROLLER_PORT, this);
        codriverController = new CoDriverController(IOConstants.CODRIVER_CONTROLLER_PORT, this);
        homingController = new HomingController(IOConstants.HOMING_CONTROLLER_PORT, this);

        // init subsystems
        ledSubsystem = new LEDSubsystem(new PWMSparkMax(PWMConstants.LED_PWM));
        armSubsystem = new ArmSubsystem();
        driveSubsystem = new DriveSubsystem(this);
        clawSubsystem = new ClawSubsystem(this);

        // init distance sensor
        distanceSensor = new DistanceSensor(AnalogConstants.DISTANCE_SENSOR);

        /*driverController.button(1).onTrue(Commands.runOnce(() -> {

        }));*/
        
        // init controller bindings
        driverController.initBindings();
        codriverController.initBindings();
        homingController.initBindings();

        // initialize auto path chooser
        String pathWeaverPath = Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner";
        System.out.println(pathWeaverPath);
        String[] paths = new File(pathWeaverPath).list();
        for (int i = 0 ; i < paths.length; i++) {
            String path = "";
            int j = paths[i].length() - 1;
            for (; j >= 0; j--) {
                if (paths[i].charAt(j) == '.') {
                    break;
                }
            }

            path = paths[i].substring(0, j);

            System.out.println(path);
            autoSelector.addOption(path, path);
        }

        autoSelector.setDefaultOption(defaultAutoPath, defaultAutoPath);

        SmartDashboard.putData(autoSelector);

        // init cam server
        CameraServer.startAutomaticCapture();
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        System.out.println("GETTING: " + autoSelector.getSelected());

        if (autoSelector.getSelected() == "none") {
            return null;
        }

        return driveSubsystem.followTrajectoryCommand(
            PathPlanner.loadPath(
                autoSelector.getSelected(),
                new PathConstraints(3, 1)
            ),
            true
        );
        // An example command will be run in autonomous
        //return Autos.exampleAuto(exampleSubsystem);
    }
}
