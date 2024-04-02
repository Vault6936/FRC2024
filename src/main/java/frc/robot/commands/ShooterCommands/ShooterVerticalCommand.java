package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterVerticalCommand extends Command {
    final ShooterSubsystem subsystem = ShooterSubsystem.getInstance();
    DoubleSupplier input;
    public ShooterVerticalCommand(DoubleSupplier in)
    {
        input = in;
        addRequirements(subsystem);
    }
    @Override
    public void execute()
    {
        subsystem.setVertical(input.getAsDouble());
    }
}
