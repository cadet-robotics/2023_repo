package frc.robot.commands.homing;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.IOConstants.HomingController;

public class RotateWheelCommand extends CommandBase {
    private CANSparkMax motor; 
    private CommandPS4Controller homingController;

    public RotateWheelCommand(CANSparkMax motor, CommandPS4Controller homingController) {
        this.motor = motor;
        this.homingController = homingController;
    }

    @Override
    public void execute() {
        if (homingController.button(HomingController.COARSE_HOMING_SPEED).getAsBoolean()) {
            motor.set(NeoMotorConstants.HOMING_SPEED_COARSE);
        } else {
            motor.set(NeoMotorConstants.HOMING_SPEED_FINE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
