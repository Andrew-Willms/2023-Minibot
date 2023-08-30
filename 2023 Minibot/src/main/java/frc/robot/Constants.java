// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.config.SwerveModuleConstants;



public final class Constants {

	public static class OperatorConstants {
		public static final int DriverControllerPort = 0;
	}

	public static final class Swerve {

		public static final double StickDeadband = 0.1;

		public static final int GyroId = 6;
		public static final boolean InvertGyro = false; // Always ensure Gyro is CCW+ CW-

		public static final double TrackWidth = Units.inchesToMeters(15 - 2 * 2.625);
		public static final double WheelBase = Units.inchesToMeters(16 - 2 * 2.625);
		public static final double WheelDiameter = Units.inchesToMeters(4.0);
		public static final double WheelCircumference = WheelDiameter * Math.PI;
	
		public static final double OpenLoopRamp = 0.25;
		public static final double ClosedLoopRamp = 0.0;

		public static final double DriveRatio = (48.0 / 40.0) * (19.0 / 25.0) * (45.0 / 15.0); // 2.736:1
		public static final double AzimuthRatio = (50.0 / 14.0) * (60.0 / 10.0); // 150:7

		public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
			new Translation2d(WheelBase / 2.0, TrackWidth / 2.0),
			new Translation2d(WheelBase / 2.0, TrackWidth / 2.0),
			new Translation2d(-WheelBase / 2.0, TrackWidth / 2.0),
			new Translation2d(-WheelBase / 2.0, -TrackWidth / 2.0)
		);

		// Swerve Voltage Compensation
		public static final double VoltageCompensation = 12.0;

		// Swerve Current Limiting
		public static final int AzimuthContinuousCurrentLimit = 30;
		public static final int DriveContinuousCurrentLimit = 60;

		// Angle Motor PID Values
		public static final double AngleKP = 0.01;
		public static final double AngleKI = 0.0;
		public static final double AngleKD = 0.0;
		public static final double AngleKFF = 0.0;

		// Drive Motor PID Values
		public static final double DriveKP = 0.1;
		public static final double DriveKI = 0.0;
		public static final double DriveKD = 0.0;
		public static final double DriveKFF = 0.0;

		// Drive Motor Characterization Values
		public static final double DriveKS = 0.667;
		public static final double DriveKV = 2.44;
		public static final double DriveKA = 0.27;

		// Drive Motor Conversion Factors
		public static final double DriveConversionPositionFactor = (WheelDiameter * Math.PI) / DriveRatio;
		public static final double DriveConversionVelocityFactor = DriveConversionPositionFactor / 60.0;
		public static final double AngleConversionFactor = 360.0 / AzimuthRatio;

		// Swerve Profiling Values
		public static final double MaxSpeed = 11; // meters per second
		public static final double MaxAngularVelocity = 11.5;

		// Neutral Modes
		public static final IdleMode AngleNeutralMode = IdleMode.kBrake;
		public static final IdleMode DriveNeutralMode = IdleMode.kBrake;

		// Motor Inverts
		public static final boolean DriveInvert = true;
		public static final boolean AngleInvert = true;

		// Angle Encoder Invert
		public static final boolean CANCoderInvert = false;

		// Front Left Module - Module 0
		public static final class Module0 {
			public static final int DriveMotorID = 1;
			public static final int AzimuthMotorID = 2;
			public static final int CANCoderID = 2;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(325.13);
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(DriveMotorID, AzimuthMotorID, CANCoderID, angleOffset);
		}

		// Front Right Module - Module 1
		public static final class Module1 {
			public static final int DriveMotorID = 3;
			public static final int AzimuthMotorID = 4;
			public static final int CANCoderID = 3;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(237.5);
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(DriveMotorID, AzimuthMotorID, CANCoderID, angleOffset);
		}

		// Back Left Module - Module 2
		public static final class Moduel2 {
			public static final int DriveMotorID = 7;
			public static final int AzimuthMotorID = 8;
			public static final int CANCoderID = 0;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(33.87);
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(DriveMotorID, AzimuthMotorID, CANCoderID, angleOffset);
		}

		// Back Right Module - Module 3
		public static final class Module3 {
			public static final int DriveMotorID = 5;
			public static final int AzimuthMotorID = 6;
			public static final int CADCoderID = 1;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(294.181);
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(DriveMotorID, AzimuthMotorID, CADCoderID, angleOffset);
		}
		
	}

}