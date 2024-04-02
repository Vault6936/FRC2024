package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterIntakeCommand extends Command {
    final ShooterSubsystem subsystem = ShooterSubsystem.getInstance();
    final MotorDirection dir;
    final DoubleSupplier speed;
    public ShooterIntakeCommand(MotorDirection direction)
    {
        this(direction, () -> (-1));
    }
    public ShooterIntakeCommand(MotorDirection direction, DoubleSupplier speed)
    {
        dir = direction;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void execute()
    {
        subsystem.setIntakeMotor(dir);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setIntakeMotor(MotorDirection.MOTOR_STOP);
    }
}
