package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommands.climberDefaultCommand;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.ShooterCommands.*;
import frc.robot.commands.IntakeToShooterCommands.*;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.*;

public class RobotContainer {
    boolean USE_SINGLE_CONTROLLER = true;

    public final CommandSwitchController payload = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);
    private final CommandJoystick baseController = new CommandJoystick(OperatorConstants.DRIVER_CONTROLLER_PORT);
    public final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    public final IntakeToShooterSubsystem intakeSubsystem = IntakeToShooterSubsystem.getInstance();
    public final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    //public final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
    private final DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(() -> -baseController.getRawAxis(1), () -> baseController.getRawAxis(0), () -> baseController.getRawAxis(2), () -> baseController.getRawAxis(3));
    // gamepad control
    //private final DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(() -> baseController.getLeftX(), () -> baseController.getLeftY(), () -> baseController.getRightX());
//    public final IntakeVerticalCommand intakeVerticalCommand;
//    public final ShooterVerticalCommand shooterVerticalCommand;
//    public final climberDefaultCommand climberDefaultCommand;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        if(USE_SINGLE_CONTROLLER) {
            baseController.button(6).whileTrue(new ShooterLaunchCommand(shooterSubsystem, MotorDirection.MOTOR_FORWARD));
            baseController.button(1).whileTrue(new ShooterLaunchCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD));
            baseController.button(12).whileTrue(new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_FORWARD));
            //baseController.button(2).whileTrue(new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD));
            baseController.button(3).whileTrue(new IntakeCommand(intakeSubsystem, MotorDirection.MOTOR_FORWARD));
            baseController.button(4).whileTrue(new IntakeCommand(intakeSubsystem, MotorDirection.MOTOR_BACKWARD));

            baseController.button(8).whileTrue(new IntakeVerticalCommand(intakeSubsystem, () -> 2.5));
            baseController.button(7).whileTrue(new IntakeVerticalCommand(intakeSubsystem, () -> -2.5));
            baseController.button(13).whileTrue(new ShooterVerticalCommand(shooterSubsystem, () -> 1.5));
            baseController.button(14).whileTrue(new ShooterVerticalCommand(shooterSubsystem, () -> -1.5));

            baseController.button(10).onTrue(new ParallelCommandGroup(new InstantCommand(driveSubsystem::resetGyro),
                    new InstantCommand(intakeSubsystem::resetIntakePosition)));


            //baseController.button(1).whileTrue(new DriveToPosition(driveSubsystem, new Pose2d(0, 0, Rotation2d.fromDegrees(45))));

//            baseController.button(2).whileTrue(new SequentialCommandGroup(
//                    new IntakeMoveToPos(Constants.PositionConstants.INTAKE_OUT_POSITION, intakeSubsystem),
//                    new IntakeCommand(intakeSubsystem, MotorDirection.MOTOR_BACKWARD, 3),
//                    new IntakeMoveToPos(Constants.PositionConstants.INTAKE_IN_POSITION, intakeSubsystem)
//            ));
//            baseController.button(1).whileTrue(new SequentialCommandGroup(
//                    new IntakeMoveToPos(Constants.PositionConstants.INTAKE_TRANSFER_POSITION, intakeSubsystem),
//                    new MoveShooterToPos(Constants.PositionConstants.SHOOTER_TRANSFER_POSITION, shooterSubsystem),
//                    new WaitCommand(0.5),
//                    new ParallelCommandGroup(
//                            new IntakeCommand(intakeSubsystem, MotorDirection.MOTOR_FORWARD, 2),
//                            new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD, 2)),
//                    new ShooterLaunchCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD)));
        }
        else {
//            payload.button().whileTrue(new ShooterLaunchCommand(shooterSubsystem, MotorDirection.MOTOR_FORWARD));
//            payload.button().whileTrue(new ShooterLaunchCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD));
//            payload.button().whileTrue(new IntakeCommand(intakeToShooterSubsystem, MotorDirection.MOTOR_FORWARD));
//            payload.button().whileTrue(new IntakeCommand(intakeToShooterSubsystem, MotorDirection.MOTOR_BACKWARD));
//            payload.button().whileTrue(new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_FORWARD));
//            payload.button().whileTrue(new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD));

//            intakeSubsystem.setDefaultCommand(intakeVerticalCommand);
//            shooterSubsystem.setDefaultCommand(shooterVerticalCommand);
        }
        //climbSubsystem.setDefaultCommand(climberDefaultCommand);
        driveSubsystem.setDefaultCommand(driveDefaultCommand);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
