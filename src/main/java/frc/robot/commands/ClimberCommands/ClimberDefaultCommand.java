package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

import java.util.function.DoubleSupplier;

public class ClimberDefaultCommand extends Command {
    ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
    DoubleSupplier leftY;
    DoubleSupplier rightY;

    public ClimberDefaultCommand(DoubleSupplier leftY, DoubleSupplier rightY){
        this.leftY = leftY;
        this.rightY = rightY;
        addRequirements(climbSubsystem);
    }
    @Override
    public void execute(){
        climbSubsystem.setClimbPos(leftY.getAsDouble(),rightY.getAsDouble());
    }
}
