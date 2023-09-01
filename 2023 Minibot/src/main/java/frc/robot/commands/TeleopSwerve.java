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

	private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
	private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
	private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

	public TeleopSwerve(
		SwerveDrive swerveDrive,
		DoubleSupplier translationSupplier,
		DoubleSupplier strafeSupplier,
		DoubleSupplier ccwRotationSupplier,
		DoubleSupplier cwRotationSupplier,
		BooleanSupplier robotCentricSupplier) {

		addRequirements(swerveDrive);
		this.SwerveDrive = swerveDrive;
		this.TranslationSupplier = translationSupplier;
		this.StrafeSupplier = strafeSupplier;
		this.CCWRotationSupplier = ccwRotationSupplier;
		this.CWRotationSupplier = cwRotationSupplier;
		this.RobotCentricSupplier = robotCentricSupplier;
	}

	@Override
	public void execute() {

		double translationValue = translationLimiter.calculate(
			MathUtil.applyDeadband(TranslationSupplier.getAsDouble(), Constants.Swerve.StickDeadband));

		double strafeValue = strafeLimiter.calculate(
			MathUtil.applyDeadband(StrafeSupplier.getAsDouble(), Constants.Swerve.StickDeadband));

		double netRotation = CWRotationSupplier.getAsDouble() - CCWRotationSupplier.getAsDouble();
		double rotationValue = rotationLimiter.calculate(
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