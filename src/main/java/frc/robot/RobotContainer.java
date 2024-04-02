package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommands.ClimberManualCommand;
import frc.robot.commands.ClimberCommands.climberDefaultCommand;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.DriveToRelativePositionCommand;
import frc.robot.commands.ShooterCommands.*;
import frc.robot.commands.IntakeToShooterCommands.*;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final CommandJoystick baseController = new CommandJoystick(OperatorConstants.DRIVER_CONTROLLER_PORT);
    public final CommandSwitchController payload = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);
    public final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    public final IntakeToShooterSubsystem intakeSubsystem = IntakeToShooterSubsystem.getInstance();
    public final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
    private final DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(() -> -baseController.getRawAxis(1), () -> baseController.getRawAxis(0), () -> baseController.getRawAxis(2), () -> baseController.getRawAxis(3));
    public final climberDefaultCommand climberDefaultCommand = new climberDefaultCommand(climbSubsystem, () -> payload.getRawAxis(0), () -> payload.getRawAxis(2));

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        baseController.button(1).whileTrue(new ShooterLaunchCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD));
        baseController.button(2).whileTrue(new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD));
        baseController.button(3).whileTrue(new SequentialCommandGroup(
                //new IntakeMoveToPos(IntakeDirection.INTAKE_OUT, intakeSubsystem),
                new IntakeCommand(intakeSubsystem, MotorDirection.MOTOR_FORWARD, 3),
                new IntakeMoveToPos(IntakeDirection.INTAKE_IN, intakeSubsystem)
        ));
        baseController.button(6).whileTrue(new ShooterLaunchCommand(shooterSubsystem, MotorDirection.MOTOR_FORWARD));
        baseController.button(4).whileTrue(new IntakeCommand(intakeSubsystem, MotorDirection.MOTOR_BACKWARD));
        baseController.button(7).whileTrue(new IntakeMoveToPos(IntakeDirection.INTAKE_IN, intakeSubsystem));
        baseController.button(8).whileTrue(new IntakeMoveToPos(IntakeDirection.INTAKE_OUT, intakeSubsystem));

        baseController.button(10).onTrue(new InstantCommand(driveSubsystem::resetGyro));
        //baseController.button(12).whileTrue(new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_FORWARD));

        baseController.button(13).whileTrue(new ShooterVerticalCommand(shooterSubsystem, () -> 1.5));
        baseController.button(14).whileTrue(new ShooterVerticalCommand(shooterSubsystem, () -> -1.5));

        baseController.button(11).whileTrue(new ClimberManualCommand(climbSubsystem, MotorDirection.MOTOR_FORWARD));
        baseController.button(16).whileTrue(new ClimberManualCommand(climbSubsystem, MotorDirection.MOTOR_BACKWARD));

        baseController.button(12).whileTrue(new ShooterExtendCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD));
        baseController.button(15).whileTrue(new ShooterExtendCommand(shooterSubsystem, MotorDirection.MOTOR_FORWARD));
        //            baseController.button(1).whileTrue(new SequentialCommandGroup(
//                    new DriveToRelativePositionCommand(driveSubsystem, new Pose2d(0.5, 0, Rotation2d.fromDegrees(0))),
//                    new DriveToRelativePositionCommand(driveSubsystem, new Pose2d(0, 0.5, Rotation2d.fromDegrees(0))),
//                    new DriveToPosition(driveSubsystem, new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
//                    new ShooterLaunchCommand(shooterSubsystem,MotorDirection.MOTOR_BACKWARD,1)
//            ));
//            baseController.button(1).whileTrue(new SequentialCommandGroup(
//                    new IntakeMoveToPos(Constants.PositionConstants.INTAKE_TRANSFER_POSITION, intakeSubsystem),
//                    new MoveShooterToPos(Constants.PositionConstants.SHOOTER_TRANSFER_POSITION, shooterSubsystem),
//                    new WaitCommand(0.5),
//                    new ParallelCommandGroup(
//                            new IntakeCommand(intakeSubsystem, MotorDirection.MOTOR_FORWARD, 2),
//                            new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD, 2)),
//                    new ShooterLaunchCommand(shooterSubsystem, MotorDirection.MOTOR_BACKWARD)));

        //climbSubsystem.setDefaultCommand(climberDefaultCommand);
        driveSubsystem.setDefaultCommand(driveDefaultCommand);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
