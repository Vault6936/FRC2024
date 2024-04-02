package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.MotorDirection;

public class ClimberManualCommand extends Command {
    final ClimbSubsystem subsystem = ClimbSubsystem.getInstance();
    final MotorDirection direction;

    public ClimberManualCommand(MotorDirection dir) {
        this.direction = dir;
        addRequirements(subsystem);
    }

    @Override
    public void execute()
    {
        switch(this.direction)
        {
            case MOTOR_BACKWARD ->
                subsystem.setClimbPower(0.8, 0.8);
            case MOTOR_FORWARD ->
                subsystem.setClimbPower(-0.8, -0.8);
        }
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setClimbPower(0,0);
    }

}
