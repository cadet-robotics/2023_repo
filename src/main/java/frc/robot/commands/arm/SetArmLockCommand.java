package frc.robot.commands.arm;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class SetArmLockCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;

    private SparkMaxAbsoluteEncoder encoder;
    
    public SetArmLockCommand(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;

        this.encoder = armSubsystem.getEncoder();

        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void execute() {
        //System.out.printf("Encoder: %s, Goal: %s\n", encoder.getPosition(), ArmConstants.MIN_POSITION + ArmConstants.LOCKING_MARGIN);
    }

    @Override
    public void initialize() {
        clawSubsystem.setClawShut(true);
        armSubsystem.setMotors(ArmConstants.LOCKING_SPEED);
    }

    @Override
    public boolean isFinished() {
        return encoder.getPosition() < ArmConstants.MIN_POSITION + ArmConstants.LOCKING_MARGIN;
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawShut(false);
        armSubsystem.setMotors(0);
    }
}
