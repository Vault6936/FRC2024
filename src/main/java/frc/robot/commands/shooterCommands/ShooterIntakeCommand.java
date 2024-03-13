package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterIntakeCommand extends Command {
    ShooterSubsystem subsystem;

    MotorDirection dir;

    public ShooterIntakeCommand(ShooterSubsystem sub, MotorDirection direction)
    {
        subsystem = sub;
        dir = direction;
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
