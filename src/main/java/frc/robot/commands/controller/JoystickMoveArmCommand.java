package frc.robot.commands.controller;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IOConstants.CoDriverControllerConsts;
import frc.robot.controllers.BaseController;
import frc.robot.subsystems.ArmSubsystem;

public class JoystickMoveArmCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private BaseController controller;
    private boolean inverse;

    public JoystickMoveArmCommand(ArmSubsystem armSubsystem, BaseController controller, boolean inverse) {
        this.armSubsystem = armSubsystem;
        this.controller = controller;
        this.inverse = inverse;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        double axisValue = controller.getRawAxis(CoDriverControllerConsts.ARM_MANUAL_AXIS);
        double inverseValue = inverse ? -1 : 1;
        
        armSubsystem.setMotors(inverseValue * axisValue * ArmConstants.MANUAL_ARM_SPEED_MAX);
    }

    /*@Override
    public boolean isFinished() {
        double axisValue = controller.getRawAxis(axis);
        return axisValue > -1 * CoDriverControllerConsts.ARM_MANUAL_DEADZONE &&
            axisValue < CoDriverControllerConsts.ARM_MANUAL_DEADZONE; 
    }*/

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotors(0);
    }
}
