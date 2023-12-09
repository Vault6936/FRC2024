package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.swerve.SwerveModule;

import java.util.ArrayList;

public class SwerveCalibrateCommand extends CommandBase {
    boolean isFinished = false;
    private ArrayList<SwerveModule<CANSparkMax>> modules;

    private DriveSubsystem subsystem = Robot.robotContainer.driveSubsystem;

    @Override
    public void initialize() {
        isFinished = false;
        modules = subsystem.getModules();
    }

    @Override
    public void execute() {
        int finished = 0;
        for (SwerveModule module : modules) {
            module.calibrateWheels();
            if (module.doneCalibrating()) finished++;
        }
        if (finished == 4) isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
