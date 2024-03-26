package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends Command {

    private final DriveSubsystem subsystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rot;
    private final DoubleSupplier speedM;

    public DriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        subsystem = DriveSubsystem.getInstance();
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.speedM = () -> (-1);
        addRequirements(subsystem);
    }
    public DriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, DoubleSupplier speedM) {
        subsystem = DriveSubsystem.getInstance();
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.speedM = speedM;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.drive(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble(), speedM.getAsDouble());
    }
}
