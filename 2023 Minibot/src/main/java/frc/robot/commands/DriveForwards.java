package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;



public class DriveForwards extends CommandBase {

	private final SwerveDrive SwerveDrive;
    private final Translation2d Translation;
    private final double Rotation;

	public DriveForwards(SwerveDrive swerveDrive, Translation2d translation, double rotation) {

		SwerveDrive = swerveDrive;
        Translation = translation;
        Rotation = rotation;
		addRequirements(swerveDrive);
	}

	@Override
	public void initialize() {

		SmartDashboard.putString("Command", "DriveForwards");
	}

	@Override
	public void execute() {

        SwerveDrive.Drive(Translation, Rotation, true, true);
    }

	@Override
	public boolean isFinished() {
		return true;
	}

}