package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utils.Direction;

public class ToggleClimberCmd extends Command {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final ClimberSubsystem climber;

	/**
	 * Creates a new Arm Raise Command.
	 *
	 * @param subsystem The arm subsystem used by this command.
	 * @param direction The direction the arm will move.
	 */
	public ToggleClimberCmd(ClimberSubsystem subsystem) {
		climber = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Call toggle once
		climber.toggleClimber();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
