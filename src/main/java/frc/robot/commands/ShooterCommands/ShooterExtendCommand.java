package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterExtendCommand extends Command {
    ShooterSubsystem subsystem;
    MotorDirection dir;
    DoubleSupplier speed;
    public ShooterExtendCommand(ShooterSubsystem sub, MotorDirection direction)
    {
        subsystem = sub;
        dir = direction;
        speed = () -> (-1);
    }
    public ShooterExtendCommand(ShooterSubsystem sub, MotorDirection direction, DoubleSupplier speed)
    {
        subsystem = sub;
        dir = direction;
        this.speed = speed;
    }
    @Override
    public void execute()
    {
        subsystem.setExtenderPower(dir);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setExtenderPower(MotorDirection.MOTOR_STOP);
    }
}
