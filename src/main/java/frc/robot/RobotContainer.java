package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.subsystems.DriveSubsystem;


public class RobotContainer {

    private final CommandPS4Controller baseController = new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> baseController.getLeftY(), () -> baseController.getRightX());

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(driveDefaultCommand);
        configureBindings();
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return null;
    }
}