package frc.robot.commands.IntakeToShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDirection;
import frc.robot.subsystems.IntakeToShooterSubsystem;

public class IntakeVericalCommand extends Command {
    private final IntakeToShooterSubsystem subsystem = IntakeToShooterSubsystem.getInstance();
    private final IntakeDirection direction;
    public IntakeVericalCommand(IntakeDirection intakeDirection){
        direction = intakeDirection;
        addRequirements(subsystem);
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        subsystem.move_intake(direction);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.move_intake(IntakeDirection.INTAKE_STOP);
    }
    @Override
    public boolean isFinished(){

        if(direction == IntakeDirection.INTAKE_READY_TO_TRANSFER_POS && subsystem.LimitSwitchIn.get())
        {
            subsystem.move_intake(IntakeDirection.INTAKE_STOP);

            return true;
        }
        if (direction == IntakeDirection.INTAKE_READY_TO_INTAKE_POS && subsystem.LimitSwitchOut.get())
        {
            subsystem.move_intake(IntakeDirection.INTAKE_STOP);

            return true;
        }
        return false;
    }
}
