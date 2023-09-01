package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.SwerveModulePosition;



public class SwerveDrive extends SubsystemBase {

	private final AHRS Gyro;

	private SwerveDriveOdometry SwerveOdometry;
	private SwerveModule[] SwerveModules;

	private Field2d field;

	public SwerveDrive() {

		Gyro = new AHRS(SerialPort.Port.kUSB); // NavX connected over MXP
		//gyro.restoreFactoryDefaults(); //for Pigeon
		ZeroGyro();

		SwerveModules = new SwerveModule[] {
			new SwerveModule(0, Constants.Swerve.Module0.constants),
			new SwerveModule(1, Constants.Swerve.Module1.constants),
			new SwerveModule(2, Constants.Swerve.Module2.constants),
			new SwerveModule(3, Constants.Swerve.Module3.constants)
		};

		SwerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
		field = new Field2d();
		SmartDashboard.putData("Field", field);
	}

	public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

		ChassisSpeeds chassisSpeed = fieldRelative
			? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
			: new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

		SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeed);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MaxSpeed);

		for (SwerveModule mod : SwerveModules) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	// Used by SwerveControllerCommand in Auto
	public void setModuleStates(SwerveModuleState[] desiredStates) {

		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MaxSpeed);

		for (SwerveModule mod : SwerveModules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	public Pose2d getPose() {
		return SwerveOdometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		SwerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
	}

//   public SwerveModuleState[] getStates() { //TODO this can probably be removed, the getmodulepositions seems to replace it
//     SwerveModuleState[] states = new SwerveModuleState[4];
//     for (SwerveModule mod : mSwerveMods) {
//       states[mod.moduleNumber] = mod.getState();
//     }
//     return states;
//   }

	public void ZeroGyro() {
		Gyro.zeroYaw();
	}

	public SwerveModulePosition[] getModulePositions() { //TODO this is new, might need to double check

		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : SwerveModules){
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public Rotation2d getYaw() {

		// return (Constants.Swerve.invertGyro)
		//     ? Rotation2d.fromDegrees(360 - gyro.getYaw())
		//     : Rotation2d.fromDegrees(gyro.getYaw());

		if (Gyro.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(Gyro.getFusedHeading());
		}
		
		// We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - Gyro.getYaw());
	}

	@Override
	public void periodic() {
		SwerveOdometry.update(getYaw(), getModulePositions());
		field.setRobotPose(getPose());

		for (SwerveModule mod : SwerveModules) {
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
		}

		// Drive(
		// 	new Translation2d(0.25, 0.25).times(Constants.Swerve.MaxSpeed),
		// 	0 * Constants.Swerve.MaxAngularVelocity,
		// 	true,
		// 	true);
	}

}