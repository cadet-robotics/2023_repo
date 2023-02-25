package frc.robot.controllers;

import frc.robot.RobotContainer;
import frc.robot.Constants.IOConstants.DriverControllerConsts;
import frc.robot.commands.arm.ManualArmDriveCommand;
import frc.robot.commands.claw.RunClawMotorCommand;

public class DriverController extends BaseController {
    public DriverController(int port, RobotContainer robotContainer) {
        super(port, robotContainer);
    }

    public void initBindings() {
        //
    }
}