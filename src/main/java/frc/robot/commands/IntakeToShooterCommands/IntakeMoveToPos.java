package frc.robot.commands.IntakeToShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeDirection;
import frc.robot.subsystems.IntakeToShooterSubsystem;
import frc.robot.subsystems.MotorDirection;

public class IntakeMoveToPos extends Command {
    private final IntakeToShooterSubsystem subsystem = IntakeToShooterSubsystem.getInstance();
    private IntakeDirection direction;
    public IntakeMoveToPos(IntakeDirection intakeDirection){
        direction = intakeDirection;
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        if (direction == IntakeDirection.INTAKE_IN) {
//            subsystem.setTargetPosition(Constants.PositionConstants.INTAKE_IN_POSITION);
            subsystem.move_intake(IntakeDirection.INTAKE_IN);
        } else {
            subsystem.move_intake(IntakeDirection.INTAKE_OUT);

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.move_intake(IntakeDirection.INTAKE_STOP);
    }
    @Override
    public boolean isFinished(){

        if(direction == IntakeDirection.INTAKE_IN && subsystem.LimitSwitchIn.get())
        {
            subsystem.move_intake(IntakeDirection.INTAKE_STOP);

            return true;
        }
        if (direction == IntakeDirection.INTAKE_OUT && subsystem.LimitSwitchOut.get())
        {
            subsystem.move_intake(IntakeDirection.INTAKE_STOP);

            return true;
        }
        return false;
    }
}
