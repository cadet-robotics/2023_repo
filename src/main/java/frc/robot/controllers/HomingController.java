package frc.robot.controllers;

import frc.robot.Constants.IOConstants.HomingControllerConsts;
import frc.robot.RobotContainer;
import frc.robot.commands.homing.ResetEncodersCommand;
import frc.robot.commands.homing.RotateWheelCommand;
import frc.robot.commands.homing.SetHomingModeCommand;
import frc.robot.commands.homing.ZeroHeadingCommand;
import frc.robot.commands.homing.ZeroSwerveCommand;
import frc.robot.subsystems.DriveSubsystem.SwerveModules;

public class HomingController extends BaseController {
    public HomingController(int port, RobotContainer robotContainer) {
        super(port, robotContainer);
    }

    public void initBindings() {
        // homing mode
        button(HomingControllerConsts.INSERT_HOMING_MODE)
            .onTrue(new SetHomingModeCommand(robotContainer, true));
        button(HomingControllerConsts.EXIT_HOMING_MODE)
            .onTrue(new SetHomingModeCommand(robotContainer, false));

        // resets
        button(HomingControllerConsts.RESET_ENCODERS_BUTTON)
            .onTrue(new ResetEncodersCommand(robotContainer.driveSubsystem));
        button(HomingControllerConsts.ZERO_HEADING_BUTTON)
            .onTrue(new ZeroHeadingCommand(robotContainer.driveSubsystem));
        button(HomingControllerConsts.ZERO_SWERVE_MODULES)
            .onTrue(new ZeroSwerveCommand(robotContainer.driveSubsystem));

        // rotate wheels
        button(HomingControllerConsts.ROTATE_FL_BUTTON)
            .whileTrue(new RotateWheelCommand(
                SwerveModules.m_frontLeft.getTurningSparkMax(),
                this
            ));
        button(HomingControllerConsts.ROTATE_FR_BUTTON)
            .whileTrue(new RotateWheelCommand(
                SwerveModules.m_frontRight.getTurningSparkMax(),
                this
            ));
        button(HomingControllerConsts.ROTATE_BL_BUTTON)
            .whileTrue(new RotateWheelCommand(
                SwerveModules.m_backLeft.getTurningSparkMax(),
                this
            ));
        button(HomingControllerConsts.ROTATE_BR_BUTTON)
            .whileTrue(new RotateWheelCommand(
                SwerveModules.m_backRight.getTurningSparkMax(),
                this
            ));
    }
}
