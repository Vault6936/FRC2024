package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveShooterToPos extends Command {
    private final ShooterSubsystem subsystem = ShooterSubsystem.getInstance();
    private double intakePos = 0;
    public MoveShooterToPos(double IntakePos){
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
        //System.out.println(Math.abs(intakePos - subsystem.findShooterPos()));
        if (Math.abs(intakePos - subsystem.findShooterPos()) <= Constants.PositionConstants.Shooter.MIN_THRESHOLD){
            return true;
        }
        return false;
    }
}
