package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommands.ClimberManualCommand;
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
    private final DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(() -> -baseController.getRawAxis(1), () -> baseController.getRawAxis(0), () -> baseController.getRawAxis(2), () -> baseController.getRawAxis(3));
    public final ShooterVerticalCommand shooterVerticalCommand = new ShooterVerticalCommand(() -> payload.getRawAxis(1));
    public final IntakeVerticalCommand intakeVerticalCommand = new IntakeVerticalCommand(() -> payload.getRawAxis(3));
    private final LimeLightSensor limelight_shooter = LimeLightSensor.getInstance();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        //(Upper joystick: small thing at the top that is a circle)
        //TODO SHOOTER LAUNCH CONTROL
        //Trigger: Shoot note out of shooter
        baseController.button(1).whileTrue(new ShooterLaunchCommand(MotorDirection.MOTOR_BACKWARD));

        //Front Middle of the Right button pad: Suck note into shooter via shooter
        baseController.button(6).whileTrue(new ShooterLaunchCommand(MotorDirection.MOTOR_FORWARD));

        //TODO SHOOTER INTAKE
        //Button BELOW the upper joystick: Robot's shooter intake intakes the outtake of the intake
        baseController.button(2).whileTrue(new ShooterIntakeCommand(MotorDirection.MOTOR_BACKWARD));


        //TODO OUTTAKE/INTAKE FULLY
        //Button LEFT of the upper joystick: Outtake to wherever via intake
        baseController.button(3).whileTrue(new IntakeCommand(MotorDirection.MOTOR_FORWARD));

        //Button RIGHT of the upper joystick: Lower Intake; Intake; Put Intake into position to outtake to shooter
        baseController.button(4).whileTrue(getIntakeCommand());

        //TODO INTAKE VERTICAL
        //Front Left of the Right button pad: Move Intake vertically up
        baseController.button(7).whileTrue(new IntakeVericalCommand(IntakeDirection.INTAKE_READY_TO_TRANSFER_POS));

        //Bottom Left of the Right button pad: Move Intake vertically down
        baseController.button(8).whileTrue(new IntakeVericalCommand(IntakeDirection.INTAKE_READY_TO_INTAKE_POS));


        //TODO GYRO RESET
        //Bottom Right of the Right button pad: Reset Gyro
        baseController.button(10).onTrue(new InstantCommand(DriveSubsystem.getInstance()::resetGyro));


        //TODO SHOOTER AIM
        //Front Right of the Left button pad: Raise shooter aim
        baseController.button(13).whileTrue(new ShooterVerticalCommand(() -> 1.5));

        //Bottom Right of the Left button pad: Lower shooter aim
        baseController.button(14).whileTrue(new ShooterVerticalCommand(() -> -1.5));


        //TODO CLIMBER
        //Front Left of the Left button pad: Raise Climber
        baseController.button(11).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_FORWARD));

        //Bottom Left of the Left button pad: Lower Climber
        baseController.button(16).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_BACKWARD));


        //TODO AMP SCORE STICK
        //Front Middle of the Left button pad
        baseController.button(12).whileTrue(new ShooterExtendCommand(MotorDirection.MOTOR_BACKWARD));

        //Bottom Middle of the Left button pad
        baseController.button(15).whileTrue(new ShooterExtendCommand(MotorDirection.MOTOR_FORWARD));























        payload.y().whileTrue(getAmpShootCommand());

        payload.a().whileTrue(getSpeakerShootCommand(false));

        payload.b().whileTrue(getTransferCommand());
        payload.x().whileTrue(getIntakeCommand());
        payload.plus().whileTrue(new ShooterIntakeCommand(MotorDirection.MOTOR_BACKWARD));
        payload.minus().whileTrue(new ShooterIntakeCommand(MotorDirection.MOTOR_FORWARD));
        payload.home().whileTrue(new IntakeCommand(MotorDirection.MOTOR_BACKWARD));
        payload.screenshot().whileTrue(new IntakeCommand(MotorDirection.MOTOR_FORWARD));

        payload.l().and(payload.r().negate()).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_FORWARD, ClimbMotors.LEFT));
        payload.r().and(payload.l().negate()).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_FORWARD, ClimbMotors.RIGHT));
        payload.r().and(payload.l()).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_FORWARD, ClimbMotors.BOTH));
        payload.zl().and(payload.zr().negate()).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_BACKWARD, ClimbMotors.LEFT));
        payload.zr().and(payload.zl().negate()).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_BACKWARD, ClimbMotors.RIGHT));
        payload.zr().and(payload.zl()).whileTrue(new ClimberManualCommand(MotorDirection.MOTOR_BACKWARD, ClimbMotors.BOTH));

        payload.pov(180).whileTrue(new ShooterLaunchCommand(MotorDirection.MOTOR_FORWARD, () -> 0.2));
        payload.pov(0).whileTrue(getShooterSourceIntakeCommand());
        payload.pov(270).whileTrue(getShooterTravelCommand());



        DriveSubsystem.getInstance().setDefaultCommand(driveDefaultCommand);
        IntakeToShooterSubsystem.getInstance().setDefaultCommand(intakeVerticalCommand);
        ShooterSubsystem.getInstance().setDefaultCommand(shooterVerticalCommand);
    }



    public Command getTransferCommand()
    {
        return new SequentialCommandGroup(
                new InstantCommand(LimeLightSensor.getInstance()::setAmpTargetID),
                new IntakeVericalCommand(IntakeDirection.INTAKE_READY_TO_TRANSFER_POS),
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
                new ShooterExtendCommand(MotorDirection.MOTOR_FORWARD, 1),
                new ShooterLaunchCommand(MotorDirection.MOTOR_BACKWARD, () -> -0.5,  0.5),
                new ShooterExtendCommand(MotorDirection.MOTOR_BACKWARD, 1)
        );
    }

    public Command getSpeakerShootCommand(boolean includeAutoDrive)
    {
        Command possibleDriveToLocation;
        if(includeAutoDrive)
        {
            possibleDriveToLocation = new DriveToPosition(limelight_shooter.getSpeakerTargetPosition());
        }
        else
        {
            possibleDriveToLocation = new InstantCommand(); // do nothing, instantly
        }

        return new SequentialCommandGroup(
                new InstantCommand(LimeLightSensor.getInstance()::setSpeakerTargetID),
                new ParallelCommandGroup(
                        new MoveShooterToPos(Constants.PositionConstants.Shooter.LAUNCH_SPEAKER),
                        possibleDriveToLocation
                ),
                new WaitCommand(0.5),
                new ShooterLaunchCommand(MotorDirection.MOTOR_BACKWARD, 2)
        );
    }

    public Command getIntakeCommand()
    {
        return new SequentialCommandGroup(
                new IntakeVericalCommand(IntakeDirection.INTAKE_READY_TO_INTAKE_POS),
                //Intake
                new IntakeCommand(MotorDirection.MOTOR_BACKWARD, 8),
                new IntakeVericalCommand(IntakeDirection.INTAKE_READY_TO_TRANSFER_POS)
        );
    }

    public Command getShooterTravelCommand()
    {
        return new MoveShooterToPos(Constants.PositionConstants.Shooter.TRAVEL_POSITION);
    }

    public Command getShooterSourceIntakeCommand()
    {
        return new MoveShooterToPos(Constants.PositionConstants.Shooter.INTAKE_SOURCE);
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                //new InstantCommand(() -> DriveSubsystem.getInstance().resetGyro(Constants.PositionConstants.PODIUM_START_ANGLE)),
                getSpeakerShootCommand(false),
          new DriveToRelativePositionCommand(new Pose2d(new Translation2d(-2, 0), Rotation2d.fromDegrees(0)))
        );
    }
}
