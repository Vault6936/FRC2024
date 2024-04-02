package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

import java.util.function.DoubleSupplier;

public class climberDefaultCommand extends Command {
    ClimbSubsystem  climbSubsystem                                  ;

    DoubleSupplier leftY;
    DoubleSupplier rightY;

    public climberDefaultCommand(ClimbSubsystem climbSubsystem, DoubleSupplier leftY, DoubleSupplier rightY){
        this.climbSubsystem = climbSubsystem;
        this.leftY = leftY;
        this.rightY = rightY;
        addRequirements(climbSubsystem);
    }
    @Override
    public void execute(){
        climbSubsystem.setClimbPos(leftY.getAsDouble(),rightY.getAsDouble());
    }
}
