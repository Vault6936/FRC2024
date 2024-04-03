package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommands.ClimberManualCommand;
import frc.robot.commands.ClimberCommands.ClimberDefaultCommand;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.ShooterCommands.*;
import frc.robot.commands.IntakeToShooterCommands.*;
import frc.robot.control.CommandSwitchController;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final CommandJoystick baseController = new CommandJoystick(OperatorConstants.DRIVER_CONTROLLER_PORT);
    public final CommandSwitchController payload = new CommandSwitchController(OperatorConstants.PAYLOAD_CONTROLLER_PORT);
    private final DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(() -> -baseController.getRawAxis(1), () -> baseController.getRawAxis(0), () -> baseController.getRawAxis(2), () -> baseController.getRawAxis(3));
    //public final ClimberDefaultCommand climberDefaultCommand = new ClimberDefaultCommand(() -> payload.getRawAxis(0), () -> payload.getRawAxis(2));
    public final ShooterVerticalCommand shooterVerticalCommand = new ShooterVerticalCommand(() -> payload.getRawAxis(1));
    public final IntakeVerticalCommand intakeVerticalCommand = new IntakeVerticalCommand(() -> payload.getRawAxis(3));

    private final LimeLightSensor limelight_shooter = new LimeLightSensor();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        baseController.button(1).whileTrue(new ShooterLaunchCommand(MotorDirection.MOTOR_BACKWARD));
        baseController.button(2).whileTrue(new ShooterIntakeCommand(MotorDirection.MOTOR_BACKWARD));
        baseController.button(3).whileTrue(new IntakeCommand(MotorDirection.MOTOR_FORWARD));
        baseController.button(4).whileTrue(getIntakeCommand());
        baseController.button(6).whileTrue(new ShooterLaunchCommand(MotorDirection.MOTOR_FORWARD));
        baseController.button(7).whileTrue(new IntakeMoveCommand(IntakeDirection.INTAKE_IN));
        baseController.button(8).whileTrue(new IntakeMoveCommand(IntakeDirection.INTAKE_OUT));

        baseController.button(10).onTrue(new InstantCommand(DriveSubsystem.getInstance()::resetGyro));
        //baseController.button(12).whileTrue(new ShooterIntakeCommand(shooterSubsystem, MotorDirection.MOTOR_FORWARD));

        baseController.button(13).whileTrue(new ShooterVerticalCommand(() -> 1.5));
        baseController.button(14).whileTrue(new ShooterVerticalCommand(() -> -1.5));

        //baseController.button(12).whileTrue(new ClimberInitializeCommand());
        baseController.button(11).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_FORWARD));
        baseController.button(16).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_BACKWARD));

        baseController.button(12).whileTrue(new ShooterExtendCommand(MotorDirection.MOTOR_BACKWARD));
        baseController.button(15).whileTrue(new ShooterExtendCommand(MotorDirection.MOTOR_FORWARD));

        payload.y().whileTrue(getAmpShootCommand());

        payload.a().whileTrue(getSpeakerShootCommand());

        payload.b().whileTrue(getTransferCommand());
        payload.x().whileTrue(getIntakeCommand());
        //climbSubsystem.setDefaultCommand(climberDefaultCommand);
        DriveSubsystem.getInstance().setDefaultCommand(driveDefaultCommand);
        IntakeToShooterSubsystem.getInstance().setDefaultCommand(intakeVerticalCommand);
        ShooterSubsystem.getInstance().setDefaultCommand(shooterVerticalCommand);
    }

    public Command getTransferCommand()
    {
        return new SequentialCommandGroup(
                new IntakeMoveCommand(IntakeDirection.INTAKE_IN),
                new MoveShooterToPos(Constants.PositionConstants.Shooter.TRANSFER_POSITION),
                new WaitCommand(0.5),
                new ShooterIntakeCommand(MotorDirection.MOTOR_BACKWARD,0.4),
                new ShooterIntakeCommand(MotorDirection.MOTOR_BACKWARD,0.8),
                new ParallelCommandGroup(
                        new IntakeCommand(MotorDirection.MOTOR_FORWARD,1),
                        new ShooterIntakeCommand(MotorDirection.MOTOR_BACKWARD, 1)
                )
                //new MoveShooterToPos(Constants.PositionConstants.Shooter.TRAVEL_POSITION)
        );
    }

    public Command getAmpShootCommand()
    {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MoveShooterToPos(Constants.PositionConstants.Shooter.LAUNCH_AMP)
                        //,new DriveToPosition();
                ),
                new ShooterExtendCommand(MotorDirection.MOTOR_FORWARD, 0.75),
                new ShooterLaunchCommand(MotorDirection.MOTOR_BACKWARD, () -> -0.5,  0.5),
                new ShooterExtendCommand(MotorDirection.MOTOR_BACKWARD, 0.76)
        );
    }

    public Command getSpeakerShootCommand()
    {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MoveShooterToPos(Constants.PositionConstants.Shooter.LAUNCH_SPEAKER)
                        //,new DriveToPosition();
                ),
                new WaitCommand(0.5),
                new ShooterLaunchCommand(MotorDirection.MOTOR_BACKWARD, 2)
                //new MoveShooterToPos(Constants.PositionConstants.Shooter.TRAVEL_POSITION)
        );
    }

    public Command getIntakeCommand()
    {
        return new SequentialCommandGroup(
                new IntakeMoveCommand(IntakeDirection.INTAKE_OUT),
                new IntakeCommand(MotorDirection.MOTOR_BACKWARD, 8),
                new IntakeMoveCommand(IntakeDirection.INTAKE_IN)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
