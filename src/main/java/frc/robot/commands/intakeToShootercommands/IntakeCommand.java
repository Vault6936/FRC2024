// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeToShootercommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeToShooterSubsystem;
import frc.robot.subsystems.MotorDirection;


/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeToShooterSubsystem subsystem;

    private MotorDirection direction;
    

    public IntakeCommand(IntakeToShooterSubsystem sub, MotorDirection dir)
    {
        direction = dir;
        subsystem = sub;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.intake(direction);
    }
    
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.intake(MotorDirection.MOTOR_STOP);
    }
    
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
