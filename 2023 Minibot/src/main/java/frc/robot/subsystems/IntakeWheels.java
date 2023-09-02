package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;



public class IntakeWheels extends SubsystemBase {

	private final CANSparkMax LeftMotor;
	private final CANSparkMax RightMotor;

	private double Power = Constants.IntakeWheels.Power.Rest.Value;

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

		SetPower(Power);
	}

	public double GetPower() {
		return Power;
	}

	public void SetPower(double power) {

		Power = power;
		LeftMotor.set(power);
		RightMotor.set(power);
	}

	@Override
	public void periodic() { }

}