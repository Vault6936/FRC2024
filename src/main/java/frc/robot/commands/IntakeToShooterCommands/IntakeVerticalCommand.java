package frc.robot.commands.IntakeToShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeToShooterSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeVerticalCommand extends Command {
    final IntakeToShooterSubsystem subsystem = IntakeToShooterSubsystem.getInstance();
    DoubleSupplier input;
    public IntakeVerticalCommand(DoubleSupplier in)
    {
        input = in;
        addRequirements(subsystem);
    }
    @Override
    public void execute()
    {
//        if(subsystem.LimitSwitchIn.get())
//        {
//
//        }

        double move = input.getAsDouble();
        if(Math.abs(move) > 0.2) {
//            double targetPos = MathUtil.clamp(subsystem.getCurrentIntakePosition() + (move * 0.12),
//                    Constants.PositionConstants.INTAKE_OUT_POSITION, Constants.PositionConstants.INTAKE_IN_POSITION);
            subsystem.move_toPosition(move * 0.012);
        }
    }
}
