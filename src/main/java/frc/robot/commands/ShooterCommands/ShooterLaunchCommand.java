package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterLaunchCommand extends Command {
    ShooterSubsystem subsystem;
    MotorDirection dir;
    DoubleSupplier speed;
    double endTime = 0;
    double secondsToShoot = 0;
    public ShooterLaunchCommand(ShooterSubsystem sub, MotorDirection direction)
    {
        subsystem = sub;
        dir = direction;
        speed = () -> (-1);
    }
    public ShooterLaunchCommand(ShooterSubsystem sub, MotorDirection direction, DoubleSupplier speed)
    {
        subsystem = sub;
        dir = direction;
        this.speed = speed;

    }
    public ShooterLaunchCommand(ShooterSubsystem sub, MotorDirection direction, double seconds){
        subsystem = sub;
        dir = direction;
        speed = () -> (-1);
        secondsToShoot = seconds;
    }
    @Override
    public void initialize(){
        endTime = Robot.timer.get() + secondsToShoot;

    }
    @Override
    public void execute()
    {
        subsystem.setShooterMotors(dir, (speed.getAsDouble() * -0.5) + 0.5);
    }
    @Override
    public boolean isFinished(){
        return false;
//        if (endTime <= 0.01){
//            return false;
//        } else if (Math.abs(endTime-Robot.timer.get()) > .1) {
//            return false;
//        }
//        return true;
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setShooterMotors(MotorDirection.MOTOR_STOP, 0);
    }
}
