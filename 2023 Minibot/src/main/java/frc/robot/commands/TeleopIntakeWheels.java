package frc.robot.commands;

import frc.robot.subsystems.IntakeWheels;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class TeleopIntakeWheels extends CommandBase {

	private final IntakeWheels IntakeWheels;
    private final DoubleSupplier WheelSpeedSupplier;

	public TeleopIntakeWheels(IntakeWheels intakeWheels, DoubleSupplier wheelSpeedSupplier) {

        addRequirements(intakeWheels);
		IntakeWheels = intakeWheels;
        WheelSpeedSupplier = wheelSpeedSupplier;
	}

	@Override
	public void initialize() { }

	@Override
	public void execute() {
        IntakeWheels.SetSpeed(WheelSpeedSupplier.getAsDouble());
    }

	@Override
	public void end(boolean interrupted) { }

	@Override
	public boolean isFinished() {
		return false;
	}

}