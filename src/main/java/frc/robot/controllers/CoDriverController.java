package frc.robot.controllers;

import frc.robot.RobotContainer;
import frc.robot.Constants.LEDColors;
import frc.robot.Constants.IOConstants.CoDriverControllerConsts;
import frc.robot.commands.SetLightValueCommand;
import frc.robot.commands.SetWheelLockStateCommand;
import frc.robot.commands.arm.SetArmLockCommand;
import frc.robot.commands.arm.SetArmToPositionCommand;

public class CoDriverController extends BaseController {
    public CoDriverController(int port, RobotContainer robotContainer) {
        super(port, robotContainer);
    }

    public void initBindings() {
        // wheel lock (X formation)
        button(CoDriverControllerConsts.WHEEL_LOCK_BUTTON)
            .onTrue(new SetWheelLockStateCommand(robotContainer, true));
        button(CoDriverControllerConsts.WHEEL_UNLOCK_BUTTON)
            .onTrue(new SetWheelLockStateCommand(robotContainer, false));

        // led controls
        /*button(CoDriverControllerConsts.GREEN_LIGHT)
            .onTrue(new SetLightValueCommand(robotContainer.ledSubsystem, LEDColors.GREEN));
        button(CoDriverControllerConsts.RED_LIGHT)
            .onTrue(new SetLightValueCommand(robotContainer.ledSubsystem, LEDColors.RED));
        button(CoDriverControllerConsts.BLUE_LIGHT)
            .onTrue(new SetLightValueCommand(robotContainer.ledSubsystem, LEDColors.BLUE));
        button(CoDriverControllerConsts.YELLOW_LIGHT)
            .onTrue(new SetLightValueCommand(robotContainer.ledSubsystem, LEDColors.YELLOW));*/

        button(1).onTrue(new SetArmToPositionCommand(
            robotContainer.clawSubsystem, 
            robotContainer.armSubsystem, 
            0.7,
            true
        ));

        button(2).onTrue(new SetArmLockCommand(
            robotContainer.armSubsystem,
            robotContainer.clawSubsystem
        ));
    }
}
