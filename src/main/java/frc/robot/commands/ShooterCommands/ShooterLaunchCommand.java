package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterLaunchCommand extends Command {
    ShooterSubsystem subsystem = ShooterSubsystem.getInstance();
    final MotorDirection dir;
    final DoubleSupplier speed;
    double endTime = 0;
    final double secondsToShoot;
    boolean firstRun = false;

    public ShooterLaunchCommand(MotorDirection direction) {
        this(direction,  () -> (-1),  3600);
    }

    public ShooterLaunchCommand(MotorDirection direction, DoubleSupplier speed) {
        this(direction, speed, 3600);
    }

    public ShooterLaunchCommand(MotorDirection direction, double seconds) {
        this(direction, () -> (-1), seconds);
    }

    public ShooterLaunchCommand(MotorDirection direction, DoubleSupplier speed, double seconds)
    {
        this.dir = direction;
        this.speed = speed;
        this.secondsToShoot = seconds;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        firstRun = true;
    }
    @Override
    public void execute()
    {
        if(firstRun)
        {
            endTime = Robot.timer.get() + secondsToShoot;
            firstRun = false;
        }
        subsystem.setShooterMotors(dir, (speed.getAsDouble() * -0.5) + 0.5);
    }
    @Override
    public boolean isFinished(){
        return (Robot.timer.get() - endTime) > 0;
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setShooterMotors(MotorDirection.MOTOR_STOP, 0);
    }
}
