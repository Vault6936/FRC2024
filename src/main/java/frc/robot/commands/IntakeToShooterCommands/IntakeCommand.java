package frc.robot.commands.IntakeToShooterCommands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeToShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.MotorDirection;


/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command
{
    private final IntakeToShooterSubsystem subsystem = IntakeToShooterSubsystem.getInstance();
    private final MotorDirection direction;
    private final double timeToRun;
    private double timeToStop;

    private boolean wasLoadedLastTime = false;

    boolean firstRun = false;

    public IntakeCommand(MotorDirection dir)
    {
        this(dir, 3600);
    }

    public IntakeCommand(MotorDirection dir, long timeToRun)
    {
        direction = dir;
        this.timeToRun = timeToRun;
        addRequirements(subsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        firstRun = true;
    }
    
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(firstRun)
        {
            timeToStop = Robot.timer.get() + timeToRun;
            firstRun = false;
        }
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
                LEDSubsystem.getInstance().setOrange(Constants.LEDConstants.MAX_STRENGTH, 0);
                subsystem.encoder.setPosition(0.0);
                return subsystem.isLoaded();
            }
            wasLoadedLastTime = subsystem.isLoaded();
        }
        return Robot.timer.get() > timeToStop;
    }
}
