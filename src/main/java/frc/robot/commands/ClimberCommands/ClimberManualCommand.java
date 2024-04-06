package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbMotors;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.MotorDirection;

public class ClimberManualCommand extends Command {
    final ClimbSubsystem subsystem = ClimbSubsystem.getInstance();
    final MotorDirection direction;
    final ClimbMotors motors;

    public ClimberManualCommand(MotorDirection dir) {
        this(dir, ClimbMotors.BOTH);
    }

    public ClimberManualCommand(MotorDirection dir, ClimbMotors climbMotors)
    {
        this.direction = dir;
        this.motors = climbMotors;
        addRequirements(subsystem);
    }

    @Override
    public void execute()
    {
        switch(this.direction)
        {
            case MOTOR_BACKWARD ->
                subsystem.setClimbPower(0.6, 0.6, motors);
            case MOTOR_FORWARD ->
                subsystem.setClimbPower(-0.6, -0.6, motors);
        }
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setClimbPower(0,0, ClimbMotors.BOTH);
    }

}
