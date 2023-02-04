package frc.robot.commands;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.subsystems.ArmSubsystem;

// TODO: only works if the desired position is ABOVE the current position*****
public class SetArmToPositionCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double encoderPosition;

    private SparkMaxAbsoluteEncoder encoder;

    public SetArmToPositionCommand(ArmSubsystem armSubsystem, double encoderPosition) {
        this.armSubsystem = armSubsystem;
        this.encoderPosition = encoderPosition;

        this.encoder = armSubsystem.getEncoder();
    }

    @Override
    public void initialize() {
        armSubsystem.setMotors(NeoMotorConstants.ARM_SPEED);
    }

    @Override
    public boolean isFinished() {
        return encoder.getPosition() == encoderPosition;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotors(0);
    }
}
