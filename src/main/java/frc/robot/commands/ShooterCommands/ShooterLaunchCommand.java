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

    public ShooterLaunchCommand(MotorDirection direction) {
        this(direction,  () -> (-1),  3600);
    }

    public ShooterLaunchCommand(MotorDirection direction, DoubleSupplier speed) {
        this(direction, speed, 3600);
    }

    public ShooterLaunchCommand(MotorDirection direction, double seconds) {
        this(direction, () -> (-1), seconds);
    }

    private ShooterLaunchCommand(MotorDirection direction, DoubleSupplier speed, double seconds)
    {
        this.dir = direction;
        this.speed = speed;
        this.secondsToShoot = seconds;
        addRequirements(subsystem);
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
