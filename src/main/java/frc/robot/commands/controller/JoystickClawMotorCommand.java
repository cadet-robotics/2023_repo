package frc.robot.commands.controller;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.IOConstants.CoDriverControllerConsts;
import frc.robot.controllers.BaseController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class JoystickClawMotorCommand extends CommandBase {
    private ClawSubsystem clawSubsystem;
    private BaseController controller;
    private boolean inverse;

    public JoystickClawMotorCommand(ClawSubsystem clawSubsystem, BaseController controller, boolean inverse) {
        this.clawSubsystem = clawSubsystem;
        this.controller = controller;
        this.inverse = inverse;
    }

    @Override
    public void execute() {
        double axisValue = controller.getRawAxis(CoDriverControllerConsts.CLAW_MANUAL_AXIS);
        double inverseValue = inverse ? -1 : 1;
        
        clawSubsystem.setIntakeMotors(inverseValue * axisValue * ClawConstants.INTAKE_SPEED_MAX);
    }

    /*@Override
    public boolean isFinished() {
        double axisValue = controller.getRawAxis(axis);
        return axisValue > -1 * CoDriverControllerConsts.ARM_MANUAL_DEADZONE &&
            axisValue < CoDriverControllerConsts.ARM_MANUAL_DEADZONE;
    }*/

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setIntakeMotors(0);
    }
}
