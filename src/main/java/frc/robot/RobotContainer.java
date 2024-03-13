package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.intakeToShootercommands.IntakeVerticalCommand;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeToShooterSubsystem;

public class RobotContainer {

    public final Joystick payload = new Joystick(OperatorConstants.DRIVER_CONTROLLER_PORT);

    public final DriveSubsystem driveSubsystem;
    public final IntakeToShooterSubsystem intakeToShooterSubsystem = new IntakeToShooterSubsystem();
    public final IntakeVerticalCommand intakeVerticalCommand = new IntakeVerticalCommand(intakeToShooterSubsystem, () -> payload.getRawAxis(1));
    private final CommandSwitchController baseController = new CommandSwitchController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final DriveDefaultCommand driveDefaultCommand;

    public RobotContainer() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> baseController.getLeftY(), () -> baseController.getRightX());
        driveSubsystem.setDefaultCommand(driveDefaultCommand);
        configureBindings();
    }

    private void configureBindings() {
        CommandScheduler.getInstance().setDefaultCommand(intakeToShooterSubsystem, intakeVerticalCommand);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
