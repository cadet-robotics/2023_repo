package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class SetLightValueCommand extends CommandBase {
    private double value;
    private LEDSubsystem ledSubsystem;

    public SetLightValueCommand(LEDSubsystem ledSubsystem, double value) {
        this.value = value;
        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void initialize() {
        ledSubsystem.setValue(value);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
