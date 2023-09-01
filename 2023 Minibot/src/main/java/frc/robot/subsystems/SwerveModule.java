package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.config.CANCoderUtilities;
import frc.config.CANSparkMaxUtilities;
import frc.config.OnboardModuleStates;
import frc.config.SwerveModuleConstants;
import frc.config.CANCoderUtilities.CCUsage;
import frc.config.CANSparkMaxUtilities.Usage;



public class SwerveModule {

	public int moduleNumber;
	private Rotation2d lastAngle;
	private Rotation2d angleOffset;

	private CANSparkMax angleMotor;
	private CANSparkMax driveMotor;

	private RelativeEncoder driveEncoder;
	private RelativeEncoder integratedAngleEncoder;
	//private CANCoder angleEncoder;
	private final DutyCycleEncoder angleEncoderTemp;

	private final SparkMaxPIDController driveController;
	private final SparkMaxPIDController angleController;

	private final SimpleMotorFeedforward feedforward =
		new SimpleMotorFeedforward(Constants.Swerve.DriveKS, Constants.Swerve.DriveKV, Constants.Swerve.DriveKA);

	public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {

		this.moduleNumber = moduleNumber;
		angleOffset = moduleConstants.AngleOffset;

		// Angle Encoder Config
		//angleEncoder = new CANCoder(moduleConstants.CANCoderID);
		configAngleEncoder();
		angleEncoderTemp = new DutyCycleEncoder(moduleConstants.CANCoderID); //TODO remove once we have cancoders
		angleEncoderTemp.setPositionOffset(angleOffset.getDegrees() / 360); //TODO remove once we have cancoders

		// Angle Motor Config
		angleMotor = new CANSparkMax(moduleConstants.AzimuthMotorID, MotorType.kBrushless);
		integratedAngleEncoder = angleMotor.getEncoder();
		angleController = angleMotor.getPIDController();
		configAngleMotor();

		// Drive Motor Config
		driveMotor = new CANSparkMax(moduleConstants.DriveMotorID, MotorType.kBrushless);
		driveEncoder = driveMotor.getEncoder();
		driveController = driveMotor.getPIDController();
		configDriveMotor();

		lastAngle = getState().angle;
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

		// Custom optimize command, since default WPILib optimize assumes continuous controller which
		// REV and CTRE are not
		desiredState = OnboardModuleStates.optimize(desiredState, getState().angle);

		setAngle(desiredState);
		setSpeed(desiredState, isOpenLoop);
	}

	private void resetToAbsolute() {

		//System.out.println(getCanCoder().getDegrees());
		double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
		integratedAngleEncoder.setPosition(absolutePosition);
	}

	private void configAngleEncoder() {

		//angleEncoder.configFactoryDefault();
		//CANCoderUtilities.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
		//angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig); //TODO uncomment once we have cancoders
	}

	private void configAngleMotor() {
		angleMotor.restoreFactoryDefaults();
		CANSparkMaxUtilities.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
		angleMotor.setSmartCurrentLimit(Constants.Swerve.AzimuthContinuousCurrentLimit);
		angleMotor.setInverted(Constants.Swerve.AngleInvert);
		angleMotor.setIdleMode(Constants.Swerve.AngleNeutralMode);
		integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.AngleConversionFactor);
		angleController.setP(Constants.Swerve.AngleKP);
		angleController.setI(Constants.Swerve.AngleKI);
		angleController.setD(Constants.Swerve.AngleKD);
		angleController.setFF(Constants.Swerve.AngleKFF);
		angleMotor.enableVoltageCompensation(Constants.Swerve.VoltageCompensation);
		angleMotor.burnFlash();
		resetToAbsolute();
	}

	private void configDriveMotor() {

		driveMotor.restoreFactoryDefaults();
		CANSparkMaxUtilities.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
		driveMotor.setSmartCurrentLimit(Constants.Swerve.DriveContinuousCurrentLimit);
		driveMotor.setInverted(Constants.Swerve.DriveInvert);
		driveMotor.setIdleMode(Constants.Swerve.DriveNeutralMode);
		driveEncoder.setVelocityConversionFactor(Constants.Swerve.DriveConversionVelocityFactor);
		driveEncoder.setPositionConversionFactor(Constants.Swerve.DriveConversionPositionFactor);
		
		driveController.setP(Constants.Swerve.AngleKP);
		driveController.setI(Constants.Swerve.AngleKI);
		driveController.setD(Constants.Swerve.AngleKD);
		driveController.setFF(Constants.Swerve.AngleKFF);

		driveMotor.enableVoltageCompensation(Constants.Swerve.VoltageCompensation);
		driveMotor.burnFlash();
		driveEncoder.setPosition(0.0);
	}

	private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

		if (isOpenLoop) {
			double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MaxSpeed;
			driveMotor.set(percentOutput);

		} else {
			driveController.setReference(desiredState.speedMetersPerSecond, 
				ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
		}
	}

	private void setAngle(SwerveModuleState desiredState) {
		// Prevent rotating module if speed is less then 1%. Prevents jittering.
		Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MaxSpeed * 0.01))
			? lastAngle
			: desiredState.angle;

		angleController.setReference(angle.getDegrees(), ControlType.kPosition);
		lastAngle = angle;
	}

	private Rotation2d getAngle() {
		return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
	}

	public Rotation2d getCanCoder() {

		return Rotation2d.fromDegrees(angleEncoderTemp.getAbsolutePosition() * 360);
		//TODO may need the if statements with the offsets here like we had in the other code

		//return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()); //TODO uncomment once we have can coders
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
	}

	public SwerveModulePosition getPosition() {

		return new SwerveModulePosition(
				driveEncoder.getPosition(), //TODO this line is kinda sus, its for falcons might need to change for neos
				getAngle()
		);
	}

}