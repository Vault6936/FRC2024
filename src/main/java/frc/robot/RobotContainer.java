package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.swerve.AngleHelpers;
import frc.robot.swerve.SwerveModule;
import frc.robot.webdashboard.DashboardLayout;

import java.util.function.DoubleSupplier;


public class RobotContainer {

    private final CommandSwitchController baseController = new CommandSwitchController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    public final DriveSubsystem driveSubsystem;

    private final DriveDefaultCommand driveDefaultCommand;

    public RobotContainer() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> baseController.getLeftY(), () -> -baseController.getRightX());
        driveSubsystem.setDefaultCommand(driveDefaultCommand);
        configureBindings();
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
