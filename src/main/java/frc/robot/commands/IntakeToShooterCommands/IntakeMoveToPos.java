package frc.robot.commands.IntakeToShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeToShooterSubsystem;

public class IntakeMoveToPos extends Command {
    private final IntakeToShooterSubsystem subsystem = IntakeToShooterSubsystem.getInstance();

    private final double targetPos;

    public IntakeMoveToPos(double targetPos)
    {
        this.targetPos = targetPos;
    }

    @Override
    public void execute()
    {
        subsystem.move_toPosition(targetPos);
    }

    @Override
    public boolean isFinished()
    {
        return (Math.abs(subsystem.getCurrentIntakePosition() - targetPos) < Constants.PositionConstants.MIN_POSITION_THRESHOLD);
    }
}
