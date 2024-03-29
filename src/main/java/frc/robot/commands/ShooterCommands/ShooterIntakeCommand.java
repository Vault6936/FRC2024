package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterIntakeCommand extends Command {
    ShooterSubsystem subsystem;
    MotorDirection dir;
    DoubleSupplier speed;
    public ShooterIntakeCommand(ShooterSubsystem sub, MotorDirection direction)
    {
        subsystem = sub;
        dir = direction;
        speed = () -> (-1);
    }
    public ShooterIntakeCommand(ShooterSubsystem sub, MotorDirection direction, DoubleSupplier speed)
    {
        subsystem = sub;
        dir = direction;
        this.speed = speed;
    }
    @Override
    public void execute()
    {
        subsystem.setShooterMotors(dir, (speed.getAsDouble() * -0.5) + 0.5);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setShooterMotors(MotorDirection.MOTOR_STOP, 0);
    }
}
