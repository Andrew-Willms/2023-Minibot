package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class TeleopSwerve extends CommandBase {

	private SwerveDrive SwerveDrive;
	private DoubleSupplier TranslationSupplier;
	private DoubleSupplier StrafeSupplier;
	private DoubleSupplier CCWRotationSupplier;
	private DoubleSupplier CWRotationSupplier;
	private BooleanSupplier RobotCentricSupplier;

	private SlewRateLimiter TranslationLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.Translation);
	private SlewRateLimiter StrafeLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.Strafe);
	private SlewRateLimiter RotationLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.RotationLimit);
	
	public TeleopSwerve(
		SwerveDrive swerveDrive,
		DoubleSupplier translationSupplier,
		DoubleSupplier strafeSupplier,
		DoubleSupplier ccwRotationSupplier,
		DoubleSupplier cwRotationSupplier,
		BooleanSupplier robotCentricSupplier) {

		addRequirements(swerveDrive);
		SwerveDrive = swerveDrive;
		TranslationSupplier = translationSupplier;
		StrafeSupplier = strafeSupplier;
		CCWRotationSupplier = ccwRotationSupplier;
		CWRotationSupplier = cwRotationSupplier;
		RobotCentricSupplier = robotCentricSupplier;
	}

	@Override
	public void execute() {

		double translationValue = TranslationLimiter.calculate(
			MathUtil.applyDeadband(TranslationSupplier.getAsDouble(), Constants.Swerve.StickDeadband));

		double strafeValue = StrafeLimiter.calculate(
			MathUtil.applyDeadband(StrafeSupplier.getAsDouble(), Constants.Swerve.StickDeadband));

		double netRotation = CWRotationSupplier.getAsDouble() - CCWRotationSupplier.getAsDouble();
		double rotationValue = RotationLimiter.calculate(
			MathUtil.applyDeadband(netRotation, Constants.Swerve.StickDeadband));

		SwerveDrive.Drive(
			new Translation2d(translationValue, strafeValue).times(Constants.Swerve.MaxSpeed),
			rotationValue * Constants.Swerve.MaxAngularVelocity,
			!RobotCentricSupplier.getAsBoolean(),
			true);

			SmartDashboard.putNumber("Translation", translationValue);
			SmartDashboard.putNumber("Strafe", strafeValue);
			SmartDashboard.putNumber("Rotation", netRotation);
	}

}