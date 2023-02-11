package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANConstants;

public class ArmSubsystem extends SubsystemBase {
    public CANSparkMax mainMotor = new CANSparkMax(CANConstants.MAIN_MOTOR, MotorType.kBrushless);
    private CANSparkMax helperMotor = new CANSparkMax(CANConstants.HELPER_MOTOR, MotorType.kBrushless);

    private SparkMaxAbsoluteEncoder encoder = mainMotor.getAbsoluteEncoder(Type.kDutyCycle);

    public CANSparkMax getMainMotor() { return mainMotor; }
    public CANSparkMax getHelperMotor() { return helperMotor; }
    public SparkMaxAbsoluteEncoder getEncoder() { return encoder; }

    // TODO: check if they need to be inverted
    public ArmSubsystem setMotors(double speedPWM) {
        mainMotor.set(speedPWM);
        helperMotor.set(speedPWM);

        return this;
    }

    // offsets raw encoder position to a 0-x range
    public double encoderOffsetCalc(double rawPosition) {
        return rawPosition + (ArmConstants.ENCODER_MAX - ArmConstants.ARM_MIN_POSITION);
    }

    // removes encoder offset 
    public double offsetToRawCalc(double rawPosition) {
        return rawPosition - (ArmConstants.ENCODER_MAX - ArmConstants.ARM_MIN_POSITION);
    }

    @Override
    public void periodic() {
        
    }
}
