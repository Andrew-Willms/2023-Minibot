package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;



public class IntakeWheels extends SubsystemBase {

	private final CANSparkMax LeftMotor;
	private final CANSparkMax RightMotor;

	private final SparkMaxPIDController LeftPIDController;
	private final SparkMaxPIDController RightPIDController;

	private Constants.IntakeWheels.Target Target = Constants.IntakeWheels.Target.Rest;

	public IntakeWheels() {

		LeftMotor = new CANSparkMax(Constants.IntakeWheels.LeftMotorCANID, MotorType.kBrushless);
		RightMotor = new CANSparkMax(Constants.IntakeWheels.RightMotorCANID, MotorType.kBrushless);

		LeftMotor.setSecondaryCurrentLimit(Constants.IntakeWheels.LeftMotorSecondaryCurrentLimit);
		RightMotor.setSecondaryCurrentLimit(Constants.IntakeWheels.RightMotorSecondaryCurrentLimit);

		LeftMotor.setSmartCurrentLimit(Constants.IntakeWheels.LeftMotorSmartCurrentLimit);
		RightMotor.setSmartCurrentLimit(Constants.IntakeWheels.RightMotorSmartCurrentLimit);

		LeftMotor.setInverted(Constants.IntakeWheels.LeftMotorIsInverted);
		RightMotor.setInverted(Constants.IntakeWheels.RightMotorIsInverted);

		LeftMotor.setIdleMode(IdleMode.kBrake);
		RightMotor.setIdleMode(IdleMode.kBrake);

		LeftPIDController = LeftMotor.getPIDController();
		LeftPIDController.setP(Constants.IntakeWheels.LeftMotorP);
		LeftPIDController.setI(Constants.IntakeWheels.LeftMotorI);
		LeftPIDController.setIZone(Constants.IntakeWheels.LeftMotorIZone);
		LeftPIDController.setD(Constants.IntakeWheels.LeftMotorD);
		LeftPIDController.setFF(Constants.IntakeWheels.LeftMotorFeedForward);

		RightPIDController = RightMotor.getPIDController();
		RightPIDController.setP(Constants.IntakeWheels.RightMotorP);
		RightPIDController.setI(Constants.IntakeWheels.RightMotorI);
		RightPIDController.setIZone(Constants.IntakeWheels.RightMotorIZone);
		RightPIDController.setD(Constants.IntakeWheels.RightMotorD);
		RightPIDController.setFF(Constants.IntakeWheels.RightMotorFeedForward);

		SetTarget(Target);
	}

	public Constants.IntakeWheels.Target GetTarget() {
		return Target;
	}

	public void SetTarget(Constants.IntakeWheels.Target target) {

		Target = target;
		switch (target.TargetType) {
			
			case Power:
				LeftMotor.set(Target.Value);
				RightMotor.set(Target.Value);
				break;

			case Velocity:
				LeftPIDController.setReference(Target.Value, ControlType.kVelocity);
				RightPIDController.setReference(Target.Value, ControlType.kVelocity);
				break;
		}

	}

	@Override
	public void periodic() { }

}