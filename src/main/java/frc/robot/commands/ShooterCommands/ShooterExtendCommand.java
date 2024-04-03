package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterExtendCommand extends Command {
    final ShooterSubsystem subsystem = ShooterSubsystem.getInstance();
    final MotorDirection dir;
    final DoubleSupplier speed;
    final double duration;
    private double endTime;

    boolean firstRun = true;
    public ShooterExtendCommand(MotorDirection direction, double time)
    {
        this(direction, () -> (-1), time);
    }
    public ShooterExtendCommand(MotorDirection direction)
    {
        this(direction, () -> (-1));
    }
    public ShooterExtendCommand(MotorDirection direction, DoubleSupplier speed)
    {
        this(direction,speed,3600);
    }
    public ShooterExtendCommand(MotorDirection direction, DoubleSupplier speed, double time)
    {
        this.duration = time;
        dir = direction;
        this.speed = speed;
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
        subsystem.setExtenderPower(dir);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setExtenderPower(MotorDirection.MOTOR_STOP);
    }

    @Override
    public boolean isFinished()
    {
        return (Robot.timer.get() - endTime) > 0;
    }
}
