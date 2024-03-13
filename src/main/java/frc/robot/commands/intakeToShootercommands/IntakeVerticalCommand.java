package frc.robot.commands.intakeToShootercommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeToShooterSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeVerticalCommand extends Command {
    IntakeToShooterSubsystem subsystem;
    DoubleSupplier input;
    public IntakeVerticalCommand(IntakeToShooterSubsystem sub, DoubleSupplier in)
    {
        subsystem = sub;
        input = in;
        addRequirements(subsystem);
    }

    @Override
    public void execute()
    {
        subsystem.setVertical(input.getAsDouble());
    }
}
