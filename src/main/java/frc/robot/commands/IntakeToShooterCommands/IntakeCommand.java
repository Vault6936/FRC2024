package frc.robot.commands.IntakeToShooterCommands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeToShooterSubsystem;
import frc.robot.subsystems.MotorDirection;


/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeToShooterSubsystem subsystem;
    private MotorDirection direction;
    private final double timeToRun;
    private double timeToStop;

    private boolean wasLoadedLastTime = false;

    public IntakeCommand(IntakeToShooterSubsystem sub, MotorDirection dir)
    {
        direction = dir;
        subsystem = sub;
        this.timeToRun = 3600;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public IntakeCommand(IntakeToShooterSubsystem sub, MotorDirection dir, long timeToRun)
    {
        direction = dir;
        subsystem = sub;
        this.timeToRun = timeToRun;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timeToStop = Robot.timer.get() + timeToRun;
    }
    
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.intake(direction);
    }
    
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.intake(MotorDirection.MOTOR_STOP);
    }
    
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        if(direction == MotorDirection.MOTOR_FORWARD)
        {
            return Robot.timer.get() > timeToStop;
        }
        if(direction == MotorDirection.MOTOR_BACKWARD) {
            if (wasLoadedLastTime) {
                return subsystem.isLoaded();
            }
            wasLoadedLastTime = subsystem.isLoaded();
        }
        return false;
    }
}
