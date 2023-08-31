package frc.config;

import edu.wpi.first.math.geometry.Rotation2d;



public class SwerveModuleConstants {

	public final int DriveMotorID;
	public final int AzimuthMotorID;
	public final int CANCoderID;
	public final Rotation2d AngleOffset;

	/**
	 * Swerve Module Constants to be used when creating swerve modules.
	 *
	 * @param driveMotorID
	 * @param azimuthMotorID
	 * @param canCoderID
	 * @param angleOffset
	 */
	public SwerveModuleConstants(
		int driveMotorID, int azimuthMotorID, int canCoderID, Rotation2d angleOffset) {
		this.DriveMotorID = driveMotorID;
		this.AzimuthMotorID = azimuthMotorID;
		this.CANCoderID = canCoderID;
		this.AngleOffset = angleOffset;
	}

}