package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterIntakeCommand extends Command {
    final ShooterSubsystem subsystem = ShooterSubsystem.getInstance();
    final MotorDirection dir;
    final DoubleSupplier speed;
    final double duration;
    double endTime;
    boolean firstRun;
    public ShooterIntakeCommand(MotorDirection direction)
    {
        this(direction, () -> (-1));
    }
    public ShooterIntakeCommand(MotorDirection direction, DoubleSupplier speed)
    {
        this(direction, speed, 3600);
    }

    public ShooterIntakeCommand(MotorDirection direction, double time)
    {
        this(direction,  () -> (-1), time);
    }
    public ShooterIntakeCommand(MotorDirection direction, DoubleSupplier speed, double time)
    {
        dir = direction;
        this.speed = speed;
        this.duration = time;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        firstRun = true;
    }
    @Override
    public void execute()
    {
        if(firstRun)
        {
            endTime = Robot.timer.get() + duration;
            firstRun = false;
        }
        subsystem.setIntakeMotor(dir);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setIntakeMotor(MotorDirection.MOTOR_STOP);
    }

    @Override
    public boolean isFinished()
    {
        return Robot.timer.get() - endTime > 0;
    }
}
