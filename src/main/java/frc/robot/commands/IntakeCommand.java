package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends InstantCommand {
    private final Intake intake;
    private final String action;

    public IntakeCommand(Intake intake, String action) {
        this.intake = intake;
        this.action = action;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        switch (action) {
            case "BallIn_TubeOut":
                intake.BallIn_TubeOut();
                break;
            case "BallOut_TubeIn":
                intake.BallOut_TubeIn();
                break;
            default:
                throw new IllegalArgumentException("Invalid action: " + action);
        }
    }
}
