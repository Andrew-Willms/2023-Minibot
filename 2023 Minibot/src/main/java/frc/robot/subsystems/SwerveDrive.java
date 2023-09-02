package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.FlowControl;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;



public class SwerveDrive extends SubsystemBase {
	private final AHRS gyro;

	private SwerveDriveOdometry swerveOdometry;
	private SwerveModule[] mSwerveMods;

	private Field2d field;

	private double rotationP = 0;
	private double rotationI = 0;
	private double rotationD = 0;

	int loopcount = 0;
	int cstate = 0;

	private Relay headlights;

	private SerialPort serPort = new SerialPort(19200, Port.kMXP);

	private static double chkGyroCurrentValue = 0;
	private static double chkGyroMaxValue = -360;
	private static double chkGyroMinValue = 360;
	private static int chkGyroCnt = 0;

	public SwerveDrive() {
		//limelight.getEntry("pipeline").setNumber(1);
		gyro = new AHRS(SerialPort.Port.kUSB); // NavX connected over MXP //, (byte) 200
		//gyro.restoreFactoryDefaults(); //for Pigeon
		gyro.calibrate();
		ZeroGyro();
		

		mSwerveMods = new SwerveModule[] {
			new SwerveModule(0, Constants.Swerve.Module0.constants),
			new SwerveModule(1, Constants.Swerve.Module1.constants),
			new SwerveModule(2, Constants.Swerve.Module2.constants),
			new SwerveModule(3, Constants.Swerve.Module3.constants)
		};

		swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
		field = new Field2d();
		SmartDashboard.putData("Field", field);

		serPort.setFlowControl(FlowControl.kNone);
		serPort.setReadBufferSize(128);
		serPort.flush();
	}

	public void setRotationPID(double p, double i, double d) {
		rotationP = p;
		rotationI = i;
		rotationD = d;
	}

	public double[] getRotationPID() {
		double[] pids = {rotationP, rotationI, rotationD};
		return pids;
	}

	public void setHeadlights(boolean on) {
		if (!on) {
			headlights.set(Relay.Value.kOff);
		}
		else {
			headlights.set(Relay.Value.kForward);
		}
	}

	public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

		SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(fieldRelative
			? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
			: new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MaxSpeed);

		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}	
	}

	public void alternateDrive(double xSpeed, double ySpeed, double omegaSpeed, Pose2d robotPose2d) {
		ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation());
	}

	public void setToXOrientation() {    
		mSwerveMods[0].setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(45)), true);
		mSwerveMods[1].setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(-45)), true);
		mSwerveMods[2].setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(-45)), true);
		mSwerveMods[3].setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(45)), true);
	}

	public void resetSwerveModuleAngles() {
		for (SwerveModule mod : mSwerveMods) {
			mod.resetToAbsolute();
			mod.setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(0)), true);
		}
	}

	public void setModulesStraight() {
		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(0)), true);
		}
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MaxSpeed);

		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	public void stop() {
		//Translation2d translation = new Translation2d(0, 0);
		//drive(translation, 0, false, false);
		for(SwerveModule mod : mSwerveMods){
			mod.stopMotors();
		}
		// ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Pose2d().getRotation());
	}

	public Pose2d getPose() {
		return swerveOdometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
	}

//   public SwerveModuleState[] getStates() { //TODO this can probably be removed, the getmodulepositions seems to replace it
//     SwerveModuleState[] states = new SwerveModuleState[4];
//     for (SwerveModule mod : mSwerveMods) {
//       states[mod.moduleNumber] = mod.getState();
//     }
//     return states;
//   }

	public void ZeroGyro() {
		gyro.zeroYaw();    
	}

	public SwerveModulePosition[] getModulePositions(){ //TODO this is new, might need to double check
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for(SwerveModule mod : mSwerveMods){
				positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public Rotation2d getYaw() {
		// return (Constants.Swerve.invertGyro)
		//     ? Rotation2d.fromDegrees(360 - gyro.getYaw())
		//     : Rotation2d.fromDegrees(gyro.getYaw());

		// if (gyro.isMagnetometerCalibrated()) {
		//     // We will only get valid fused headings if the magnetometer is calibrated
		//     return Rotation2d.fromDegrees(gyro.getFusedHeading());
		//     }
		//
		//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.

		double gyroYaw = -gyro.getYaw();// + 180;
		if (gyroYaw > 180) {
			gyroYaw -= 360;
		} else if (gyroYaw < -180) {
			gyroYaw += 360;
		}
		return Rotation2d.fromDegrees(gyroYaw);
	}

	public float getGyroRoll() {
		return gyro.getRoll();
	}

	public float getGyroPitch() {
		return gyro.getPitch();
	}

	public boolean isModuleReady(int module) {
		double degrees = mSwerveMods[module].getCanCoder().getDegrees();
		if ( degrees != 0) {
			return true;
		}
		return false;
	}

	public boolean isGyroReady() {
		return gyro.isConnected();
		// double curGyro = getYaw().getDegrees();
		// if (curGyro < chkGyroMinValue) {
		//   chkGyroMinValue = curGyro;
		// }
		// else if (curGyro > chkGyroMaxValue) {
		//   chkGyroMaxValue = curGyro;
		// }
		// if (Math.abs(Math.abs(chkGyroMaxValue) - Math.abs(chkGyroMaxValue)) > 0.5) {
		//   return false;
		// }
		// if (Math.abs(getYaw().getDegrees() - chkGyroCurrentValue) != 0) {
		//   chkGyroCnt = 50;
		//   return true;
		// }
		// else {
		//   chkGyroCnt--;
		// }
		// if (chkGyroCnt == 0) {
		//   chkGyroCnt = 50;
		//   return false;
		// }
		// chkGyroCurrentValue = curGyro;
		// return false;
	}

	
	public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

		return new SequentialCommandGroup(
			new InstantCommand(() -> {
				// Reset odometry for the first path you run during auto
				if(isFirstPath){
					this.resetOdometry(traj.getInitialHolonomicPose());
					//this.ZeroGyro();
				}
			}),
			new PPSwerveControllerCommand(
				traj, 
				this::getPose, // Pose supplier
				Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
				new PIDController(4, 0, 0.3), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
				new PIDController(4, 0, 0.3), // Y controller (usually the same values as X controller)
				new PIDController(3.2, 0, 0.3), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
				this::setModuleStates, // Module states consumer
				false, 
				this // Requires this drive subsystem
			)
		);
	}

	public double getCurrentVelocity() {
		return mSwerveMods[0].getState().speedMetersPerSecond;
	}

	@Override
	public void periodic() {

		swerveOdometry.update(getYaw(), getModulePositions());
		field.setRobotPose(getPose());
		for (SwerveModule mod : mSwerveMods) {
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
			SmartDashboard.putNumber("Drive Enc " + mod.moduleNumber, mod.getDriveEncoderPosition());
		}

		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		SmartDashboard.putNumber("2dPose X", getPose().getX());
		SmartDashboard.putNumber("2dPose Y", getPose().getY());
		SmartDashboard.putNumber("Swerve Yaw", getYaw().getDegrees());
		SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
		SmartDashboard.putNumber("Wonky swerve motor", mSwerveMods[3].getSetVelocity());
		SmartDashboard.putNumber("gyro Roll", gyro.getRoll());
		SmartDashboard.putNumber("gyro Accel X", gyro.getRawAccelX());
		SmartDashboard.putNumber("gyro Accel Y", gyro.getRawAccelY());
		SmartDashboard.putNumber("gyro Accel Z", gyro.getRawAccelZ());
	}
	
}