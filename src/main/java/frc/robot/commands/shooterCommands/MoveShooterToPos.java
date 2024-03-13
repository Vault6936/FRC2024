package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveShooterToPos extends Command {
    private final ShooterSubsystem subsystem;
    private double intakePos = 0;
    public MoveShooterToPos(double IntakePos, ShooterSubsystem sub){
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
        if (Math.abs(intakePos - subsystem.findShooterPos()) <= .5 ){
            return true;
        }
        return false;
    }
}
