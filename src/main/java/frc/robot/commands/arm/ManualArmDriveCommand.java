package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmDriveCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private boolean inverse;

    public ManualArmDriveCommand(ArmSubsystem armSubsystem, boolean inverse) {
        this.armSubsystem = armSubsystem;
        this.inverse = inverse;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setMotors(ArmConstants.MANUAL_ARM_SPEED * (inverse ? - 1 : 1));
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotors(0);
    }
}
