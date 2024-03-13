package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterLaunchCommand extends Command {
    ShooterSubsystem subsystem;
    MotorDirection dir;
    public ShooterLaunchCommand(ShooterSubsystem sub, MotorDirection direction)
    {
        subsystem = sub;
        dir = direction;
    }

    @Override
    public void execute()
    {
        subsystem.setShooterMotors(dir);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setShooterMotors(MotorDirection.MOTOR_STOP);
    }
}
