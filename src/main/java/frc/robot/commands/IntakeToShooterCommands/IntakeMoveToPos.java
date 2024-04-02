package frc.robot.commands.IntakeToShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeDirection;
import frc.robot.subsystems.IntakeToShooterSubsystem;
import frc.robot.subsystems.MotorDirection;

public class IntakeMoveToPos extends Command {
    private final IntakeToShooterSubsystem subsystem;
    private IntakeDirection direction;
    private double intakeTarget;

    public IntakeMoveToPos(IntakeDirection intakeDirection, IntakeToShooterSubsystem sub){
        subsystem = sub;
        direction = intakeDirection;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
//    @Override
//    public void initialize() {
//        if (direction == IntakeDirection.INTAKE_IN) {
//            intakeTarget = Constants.PositionConstants.INTAKE_IN_POSITION;
//        } else if (direction == IntakeDirection.INTAKE_OUT) {
//            intakeTarget = Constants.PositionConstants.INTAKE_OUT_POSITION;
//        }
//    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        if (direction == IntakeDirection.INTAKE_IN) {
//            subsystem.setTargetPosition(Constants.PositionConstants.INTAKE_IN_POSITION);
            subsystem.move_intake(IntakeDirection.INTAKE_IN);
        } else {
            subsystem.move_intake(IntakeDirection.INTAKE_OUT);

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.move_intake(IntakeDirection.INTAKE_STOP);
    }
    @Override
    public boolean isFinished(){

        if(direction == IntakeDirection.INTAKE_IN && subsystem.LimitSwitchIn.get())
        {
            subsystem.move_intake(IntakeDirection.INTAKE_STOP);

            return true;
        }
        if (direction == IntakeDirection.INTAKE_OUT && subsystem.LimitSwitchOut.get())
        {
            subsystem.move_intake(IntakeDirection.INTAKE_STOP);

            return true;
        }
//        if (/*Math.abs(intakeTarget - subsystem.findIntakePos()) <= Constants.PositionConstants.MIN_POSITION_THRESHOLD*/ 1==1 ){
//            if(direction == IntakeDirection.INTAKE_IN && !subsystem.LimitSwitchIn.get())
//            {
//                subsystem.encoder.setPosition(intakeTarget - .1);
//            }
//            else if (direction == IntakeDirection.INTAKE_OUT && !subsystem.LimitSwitchOut.get()){
//                subsystem.encoder.setPosition(intakeTarget + .1);
//            } else {
//                return true;
//            }
//        }
        return false;
    }
}
