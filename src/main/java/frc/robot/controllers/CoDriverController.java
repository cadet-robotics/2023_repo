package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LEDColors;
import frc.robot.Constants.IOConstants.CoDriverControllerConsts;
import frc.robot.commands.SetLightValueCommand;
import frc.robot.commands.SetWheelLockStateCommand;
import frc.robot.commands.arm.ManualArmDriveCommand;
import frc.robot.commands.arm.SetArmLockCommand;
import frc.robot.commands.arm.SetArmToPositionCommand;
import frc.robot.commands.claw.RunClawMotorCommand;

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

        // intake motors
        axisGreaterThan(CoDriverControllerConsts.LED_MODIFIER_AXIS, CoDriverControllerConsts.LED_MODIFIER_THRESHOLD)
            .negate()
            .and(button(CoDriverControllerConsts.CLAW_SUCK))
            .whileTrue(new RunClawMotorCommand(robotContainer.clawSubsystem, true));

        axisGreaterThan(CoDriverControllerConsts.LED_MODIFIER_AXIS, CoDriverControllerConsts.LED_MODIFIER_THRESHOLD)
            .negate()
            .and(button(CoDriverControllerConsts.CLAW_VOMIT))
            .whileTrue(new RunClawMotorCommand(robotContainer.clawSubsystem, false));

        // cancels arm command
        axisGreaterThan(CoDriverControllerConsts.LED_MODIFIER_AXIS, CoDriverControllerConsts.LED_MODIFIER_THRESHOLD)
            .negate()
            .and(button(CoDriverControllerConsts.CANCEL_ARM_COMMAND))
            .onTrue(Commands.runOnce(() -> {
                Command activeCommand = robotContainer.armSubsystem.getCurrentCommand();
                if (activeCommand != null) {
                    activeCommand.cancel();
                }
            }));

        // claw open/close
        button(CoDriverControllerConsts.CLAW_CLOSE_BUTTON).onTrue(Commands.runOnce(() -> {
            robotContainer.clawSubsystem.setClawShut(true);
        }));

        button(CoDriverControllerConsts.CLAW_OPEN_BUTTON).onTrue(Commands.runOnce(() -> {
            robotContainer.clawSubsystem.setClawShut(false);
        }));

        // manual arm drive
        button(CoDriverControllerConsts.ARM_MANUAL_UP).whileTrue(
            new ManualArmDriveCommand(robotContainer.armSubsystem, true)
        );
        button(CoDriverControllerConsts.ARM_MANUAL_DOWN).whileTrue(
            new ManualArmDriveCommand(robotContainer.armSubsystem, false)
        );

        // set arm to fixed position
        pov(0).onTrue(getArmHoldSwitch(ArmConstants.ARM_PRESET_POSITIONS[0]));
        pov(90).onTrue(getArmHoldSwitch(ArmConstants.ARM_PRESET_POSITIONS[1]));
        pov(180).onTrue(getArmHoldSwitch(ArmConstants.ARM_PRESET_POSITIONS[2]));
        pov(270).onTrue(getArmHoldSwitch(ArmConstants.ARM_PRESET_POSITIONS[3]));

        // locks arm in robot
        button(CoDriverControllerConsts.ARM_LOCK).onTrue(new SetArmLockCommand(
            robotContainer.armSubsystem,
            robotContainer.clawSubsystem
        ));
    }
}
