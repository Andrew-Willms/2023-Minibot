package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;



public class StopDriving extends CommandBase {

	private final SwerveDrive SwerveDrive;

	public StopDriving(SwerveDrive swerveDrive) {

		SwerveDrive = swerveDrive;
		addRequirements(swerveDrive);
	}

	@Override
	public void initialize() {

		SmartDashboard.putString("Command", "DriveForwards");
	}

	@Override
	public void execute() {

        SwerveDrive.Drive(new Translation2d(0, 0), 0, true, true);
    }

	@Override
	public boolean isFinished() {
		return true;
	}

}