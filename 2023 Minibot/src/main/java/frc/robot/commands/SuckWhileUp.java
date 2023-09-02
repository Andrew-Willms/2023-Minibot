package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;



// An example command that uses an example subsystem.
public class SuckWhileUp extends CommandBase {

	private final IntakeWheels IntakeWheels;

	public SuckWhileUp(IntakeWheels intakeWheels) {

		IntakeWheels = intakeWheels;
		addRequirements(intakeWheels);
	}

	@Override
	public void initialize() {

		IntakeWheels.SetPower(Constants.IntakeWheels.Power.Pickup.Value);

		SmartDashboard.putString("Command", "Pickup");
	}

	@Override
	public void execute() { }

	@Override
	public boolean isFinished() {
		return true;
	}

}