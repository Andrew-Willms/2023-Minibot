package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;



public class IntakeWheels extends SubsystemBase {

	private CANSparkMax LeftIntakeMotor;
	private CANSparkMax RightIntakeMotor;

	private double IntakeWheelsSpeed = 0;

	public IntakeWheels() {

		LeftIntakeMotor = new CANSparkMax(Constants.IntakeWheels.LeftMotorCANID, MotorType.kBrushless);
		RightIntakeMotor = new CANSparkMax(Constants.IntakeWheels.RightMotorCANID, MotorType.kBrushless);

		LeftIntakeMotor.setSecondaryCurrentLimit(Constants.IntakeWheels.LeftMotorSecondaryCurrentLimit);
		RightIntakeMotor.setSecondaryCurrentLimit(Constants.IntakeWheels.RightMotorSecondaryCurrentLimit);

		LeftIntakeMotor.setSmartCurrentLimit(Constants.IntakeWheels.LeftMotorSmartCurrentLimit);
		RightIntakeMotor.setSmartCurrentLimit(Constants.IntakeWheels.RightMotorSmartCurrentLimit);

		LeftIntakeMotor.setInverted(Constants.IntakeWheels.LeftMotorIsInverted);
		RightIntakeMotor.setInverted(Constants.IntakeWheels.RightMotorIsInverted);

		LeftIntakeMotor.setIdleMode(IdleMode.kBrake);
		RightIntakeMotor.setIdleMode(IdleMode.kBrake);
	}

	public double GetSpeed() {
		return IntakeWheelsSpeed;
	}

	public void SetSpeed(double speed) {
		IntakeWheelsSpeed = speed;
	}

	@Override
	public void periodic() {
		LeftIntakeMotor.set(IntakeWheelsSpeed);
		RightIntakeMotor.set(IntakeWheelsSpeed);
	}

}