package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LEDColors;
import frc.robot.Constants.IOConstants.CoDriverControllerConsts;
import frc.robot.commands.SetLightValueCommand;
import frc.robot.commands.arm.ManualArmDriveCommand;
import frc.robot.commands.arm.SetArmLockCommand;
import frc.robot.commands.arm.SetArmToPositionCommand;
import frc.robot.commands.claw.RunClawMotorCommand;
import frc.robot.commands.controller.JoystickClawMotorCommand;
import frc.robot.commands.controller.JoystickMoveArmCommand;
import frc.robot.commands.drive.SetWheelLockStateCommand;

public class CoDriverController extends BaseController {
    public CoDriverController(int port, RobotContainer robotContainer) {
        super(port, robotContainer);
    }

    private SetArmToPositionCommand getArmHoldSwitch(double desiredPosition) {
        return new SetArmToPositionCommand(
            robotContainer.clawSubsystem,
            robotContainer.armSubsystem,
            desiredPosition,
            true
        );
    }

    public void initBindings() {
        // wheel lock (X formation)
        button(CoDriverControllerConsts.WHEEL_LOCK_BUTTON)
            .onTrue(new SetWheelLockStateCommand(robotContainer, true));
        button(CoDriverControllerConsts.WHEEL_UNLOCK_BUTTON)
            .onTrue(new SetWheelLockStateCommand(robotContainer, false));

        // led controls
        axisGreaterThan(CoDriverControllerConsts.LED_MODIFIER_AXIS, CoDriverControllerConsts.LED_MODIFIER_THRESHOLD)
            .and(button(CoDriverControllerConsts.GREEN_LIGHT))
            .onTrue(new SetLightValueCommand(robotContainer.ledSubsystem, LEDColors.GREEN));

        axisGreaterThan(CoDriverControllerConsts.LED_MODIFIER_AXIS, CoDriverControllerConsts.LED_MODIFIER_THRESHOLD)
            .and(button(CoDriverControllerConsts.RED_LIGHT))
            .onTrue(new SetLightValueCommand(robotContainer.ledSubsystem, LEDColors.RED));

        axisGreaterThan(CoDriverControllerConsts.LED_MODIFIER_AXIS, CoDriverControllerConsts.LED_MODIFIER_THRESHOLD)
            .and(button(CoDriverControllerConsts.BLUE_LIGHT))
            .onTrue(new SetLightValueCommand(robotContainer.ledSubsystem, LEDColors.BLUE));

        axisGreaterThan(CoDriverControllerConsts.LED_MODIFIER_AXIS, CoDriverControllerConsts.LED_MODIFIER_THRESHOLD)
            .and(button(CoDriverControllerConsts.YELLOW_LIGHT))
            .onTrue(new SetLightValueCommand(robotContainer.ledSubsystem, LEDColors.YELLOW));

        // claw open/close
        button(CoDriverControllerConsts.CLAW_CLOSE_BUTTON).onTrue(Commands.runOnce(() -> {
            robotContainer.clawSubsystem.setClawShut(true);
        }));

        button(CoDriverControllerConsts.CLAW_OPEN_BUTTON).onTrue(Commands.runOnce(() -> {
            robotContainer.clawSubsystem.setClawShut(false);
        }));

        // manual arm drive
        axisGreaterThan(CoDriverControllerConsts.ARM_MANUAL_AXIS, CoDriverControllerConsts.ARM_MANUAL_DEADZONE)
            .whileTrue(new JoystickMoveArmCommand(
                robotContainer.armSubsystem,
                this,
                false
            )
        );
        axisLessThan(CoDriverControllerConsts.ARM_MANUAL_AXIS, -1 * CoDriverControllerConsts.ARM_MANUAL_DEADZONE)
            .whileTrue(new JoystickMoveArmCommand(
                robotContainer.armSubsystem,
                this,
                false
            )
        );

        // claw intake motors
        axisGreaterThan(CoDriverControllerConsts.CLAW_MANUAL_AXIS, CoDriverControllerConsts.CLAW_MANUAL_DEADZONE)
            .whileTrue(new JoystickClawMotorCommand(
                robotContainer.clawSubsystem,
                this,
                true
            )
        );
        axisLessThan(CoDriverControllerConsts.CLAW_MANUAL_AXIS, -1 * CoDriverControllerConsts.CLAW_MANUAL_DEADZONE)
            .whileTrue(new JoystickClawMotorCommand(
                robotContainer.clawSubsystem,
                this,
                true
            )
        );
        
        // set arm to fixed position
        pov(0).onTrue(getArmHoldSwitch(ArmConstants.ARM_PRESET_POSITIONS[2]));
        pov(90).onTrue(getArmHoldSwitch(ArmConstants.ARM_PRESET_POSITIONS[1]));
        pov(180).onTrue(getArmHoldSwitch(ArmConstants.ARM_PRESET_POSITIONS[0]));
        pov(270).onTrue(Commands.runOnce(() -> {
            Command activeCommand = robotContainer.armSubsystem.getCurrentCommand();
            if (activeCommand != null) {
                activeCommand.cancel();
            }
        }));

        // locks arm in robot
        button(CoDriverControllerConsts.ARM_LOCK).onTrue(new SetArmLockCommand(
            robotContainer.armSubsystem,
            robotContainer.clawSubsystem
        ));
    }
}
