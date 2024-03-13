package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterVerticalCommand extends Command {
    ShooterSubsystem subsystem;
    DoubleSupplier input;
    public ShooterVerticalCommand(ShooterSubsystem sub, DoubleSupplier in)
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
