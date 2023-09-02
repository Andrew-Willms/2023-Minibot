package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class IntakeArm extends SubsystemBase {

	private final CANSparkMax Motor;
	private final RelativeEncoder Encoder;
	private final SparkMaxPIDController PIDController;

	private Constants.IntakeArm.Position TargetPosition;

	public IntakeArm(Constants.IntakeArm.Position startingPosition) {

		TargetPosition = startingPosition;

		Motor = new CANSparkMax(Constants.IntakeArm.MotorCANDID, MotorType.kBrushless);
		Motor.setIdleMode(IdleMode.kBrake);

		Motor.setSecondaryCurrentLimit(Constants.IntakeArm.MotorSecondaryCurrentLimit);
		Motor.setSmartCurrentLimit(Constants.IntakeArm.MotorSmartCurrentLimit);

		Encoder = Motor.getEncoder();
		Encoder.setPosition(0);

		PIDController = Motor.getPIDController();
		PIDController.setP(Constants.IntakeArm.MotorP);
		PIDController.setI(Constants.IntakeArm.MotorI);
		PIDController.setIZone(Constants.IntakeArm.MotorIZone);
		PIDController.setD(Constants.IntakeArm.MotorD);
		PIDController.setFF(Constants.IntakeArm.MotorFeedForward);

		PIDController.setOutputRange(-0.65, 0.65);
	}

	public void SetPosition(Constants.IntakeArm.Position position) {
		PIDController.setReference(position.EncoderPosition / 2, ControlType.kPosition); // idk why but we need to divide by 2
	}

	public Constants.IntakeArm.Position GetTargetPosition() {
		return TargetPosition;
	}

	public double GetPosition() {
		return Encoder.getPosition();
	}

	@Override
	public void periodic() {

		SmartDashboard.putNumber("Intake Position", Motor.getEncoder().getPosition());
	}

}