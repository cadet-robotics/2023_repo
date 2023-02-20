package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.commands.arm.SetArmToPositionCommand;

public class ArmSubsystem extends SubsystemBase {
    public CANSparkMax mainMotor = new CANSparkMax(CANConstants.ARM_MAIN_MOTOR, MotorType.kBrushless);
    private CANSparkMax helperMotor = new CANSparkMax(CANConstants.ARM_HELPER_MOTOR, MotorType.kBrushless);

    private SparkMaxAbsoluteEncoder encoder = mainMotor.getAbsoluteEncoder(Type.kDutyCycle);

    public CANSparkMax getMainMotor() { return mainMotor; }
    public CANSparkMax getHelperMotor() { return helperMotor; }
    public SparkMaxAbsoluteEncoder getEncoder() { return encoder; }

    public ArmSubsystem() {
        mainMotor.enableVoltageCompensation(11);
        helperMotor.enableVoltageCompensation(11);
    }

    // TODO: check if they need to be inverted
    public ArmSubsystem setMotors(double speedPWM) {
        mainMotor.set(speedPWM);
        helperMotor.set(speedPWM);

        return this;
    }

    /**
     * @return How far the arm is up in its total range, from 0.0 to 1.0
     */
    public double getRelativePosition() {
        return (encoder.getPosition() - ArmConstants.MIN_POSITION) / (ArmConstants.RANGE);
    }

    /**
     * @param relativePosition 0 to 1
     * @return The encoder value that corresponds to a given relative position
     */
    public double getAbsolutePosition(double relativePosition) {
        return relativePosition * ArmConstants.RANGE + ArmConstants.MIN_POSITION;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm/encoder", encoder.getPosition());
    }
}
