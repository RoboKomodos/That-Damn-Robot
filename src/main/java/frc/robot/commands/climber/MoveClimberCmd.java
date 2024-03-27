package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utils.Direction;

public class MoveClimberCmd extends Command {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final ClimberSubsystem climber;

    private final Direction direction;

	/**
	 * Creates a new Move Climber Command.
	 *
	 * @param subsystem The arm subsystem used by this command.
	 * @param direction The direction the arm will move.
	 */
	public MoveClimberCmd(ClimberSubsystem subsystem, Direction direction) {
		climber = subsystem;
        this.direction = direction;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Call toggle once
		climber.moveClimber(direction);
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
