package frc.robot.controllers;

import frc.robot.RobotContainer;
import frc.robot.Constants.IOConstants.DriverControllerConsts;
import frc.robot.commands.drive.AutoLevelCommand;
import frc.robot.commands.homing.ZeroHeadingCommand;

public class DriverController extends BaseController {
    public DriverController(int port, RobotContainer robotContainer) {
        super(port, robotContainer);
    }

    public void initBindings() {
        button(DriverControllerConsts.ZERO_HEADING_BUTTON)
            .onTrue(new ZeroHeadingCommand(robotContainer.driveSubsystem));

        button(DriverControllerConsts.BEGIN_AUTO_LEVEL_1)
            .and(button(DriverControllerConsts.BEGIN_AUTO_LEVEL_2))
            .onTrue(new AutoLevelCommand(robotContainer.driveSubsystem));
    }

    // gets an axis after a x^2 operation has been applied to it
    // acounts for negation
    // TODO: FIX
    public double getAxisModified(int axis) {
        double value = getRawAxis(axis);
        value *= value;
        return value > 0 ? value : -1 * value;
    }
}