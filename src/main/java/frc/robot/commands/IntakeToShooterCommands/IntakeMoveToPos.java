package frc.robot.commands.IntakeToShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeToShooterSubsystem;

public class IntakeMoveToPos extends Command {
    private final IntakeToShooterSubsystem subsystem;
    private double intakePos = 0;
    public IntakeMoveToPos(double IntakePos, IntakeToShooterSubsystem sub){
        subsystem = sub;
        intakePos = IntakePos;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        subsystem.setTargetPosition(intakePos);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished(){
        if (Math.abs(intakePos - subsystem.findIntakePos()) <= Constants.PositionConstants.MIN_POSITION_THRESHOLD ){
            return true;
        }
        return false;
    }
}
