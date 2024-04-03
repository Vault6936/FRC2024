package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimberInitializeCommand extends Command {
    ClimbSubsystem subsystem = ClimbSubsystem.getInstance();

    private boolean calibrateLeft = false;
    private boolean calibrateRight = false;
    double hasRunTime;

    @Override
    public void initialize()
    {
        calibrateRight = false;
        calibrateLeft = false;
        hasRunTime = Robot.timer.get() + 0.2;
    }
    @Override
    public void execute()
    {
        if(!calibrateLeft)
        {
            subsystem.setClimbLeft(0.4);
        }
        if(!calibrateRight)
        {
            subsystem.setClimbRight(0.4);
        }
    }
    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setClimbPower(0,0);
        subsystem.setClimbCurrentPos(0);
    }
    @Override
    public boolean isFinished()
    {
        if(Robot.timer.get() - hasRunTime > 0)
        {
            return false;
        }
        if(Math.abs(subsystem.getLeftVelocity()) < 10)
        {
            subsystem.setClimbLeft(0);
            calibrateLeft = true;
        }
        if(Math.abs(subsystem.getRightVelocity()) < 10) {
            subsystem.setClimbRight(0);
            calibrateRight = true;
        }
        return calibrateLeft && calibrateRight;
    }
}
