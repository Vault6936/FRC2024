// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeToShooterSubsystem extends SubsystemBase
{
    private static IntakeToShooterSubsystem instance;
    CANSparkMax intake = new CANSparkMax(Constants.CANIds.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
    CANSparkMax vertical = new CANSparkMax(Constants.CANIds.INTAKE_VERTICAL, CANSparkMax.MotorType.kBrushless);
    DigitalInput loadedLimitSwitchA = new DigitalInput(Constants.DigitalInputs.INTAKE_INSIDE_A);
    DigitalInput loadedLimitSwitchB = new DigitalInput(Constants.DigitalInputs.INTAKE_INSIDE_B);
    public DigitalInput LimitSwitchOut = new DigitalInput(Constants.DigitalInputs.INTAKE_OUT);
    public DigitalInput LimitSwitchIn = new DigitalInput(Constants.DigitalInputs.INTAKE_IN);
    public RelativeEncoder encoder;
    double targetPosition = 0;
    double lastResetTimer = Robot.timer.get();

    /** Creates a new ExampleSubsystem. */
    private IntakeToShooterSubsystem() {
        encoder = vertical.getEncoder();
        encoder.setPosition(0.0);
        intake.setSecondaryCurrentLimit(20);
        intake.setSmartCurrentLimit(20);
        vertical.getPIDController().setP(0.034 + .001);
        vertical.getPIDController().setOutputRange(-0.4,0.4);
    }

    public static IntakeToShooterSubsystem getInstance()
    {
        if(instance == null)
        {
            instance = new IntakeToShooterSubsystem();
        }
        return instance;
    }
    public void intake(MotorDirection dir)
    {
        switch (dir) {
            case MOTOR_FORWARD -> intake.set(1.0);
            case MOTOR_BACKWARD -> intake.set(-0.4);
            case MOTOR_STOP -> intake.set(0);
        }
    }

    public void move_intake(IntakeDirection dir)
    {
        vertical.getPIDController().setReference(0, CANSparkBase.ControlType.kDutyCycle);
        switch (dir) {
            case INTAKE_IN -> vertical.set(0.3);
            case INTAKE_OUT -> vertical.set(-0.3);
            case INTAKE_STOP -> vertical.set(0);
        }
    }

    public void control_intake(double pos)
    {
        vertical.getPIDController().setReference(0, CANSparkBase.ControlType.kDutyCycle);
        vertical.set(pos);
    }

    public void move_toPosition(double pos)
    {
        pos = MathUtil.clamp(pos, Constants.PositionConstants.INTAKE_OUT_POSITION, Constants.PositionConstants.INTAKE_IN_POSITION);
        vertical.getPIDController().setReference(pos, CANSparkBase.ControlType.kPosition);
    }

    public double getCurrentIntakePosition()
    {
        return vertical.getEncoder().getPosition();
    }

    public boolean isLoaded()
    {
        return loadedLimitSwitchA.get() || loadedLimitSwitchB.get();
    }

    @Override
    public void periodic()
    {
        if(Constants.DEBUG_INFO) {
            SmartDashboard.putBoolean("LimitSwitchIn", LimitSwitchIn.get());
            SmartDashboard.putBoolean("LimitSwitchOut", LimitSwitchOut.get());
            SmartDashboard.putBoolean("Loaded Switch", loadedLimitSwitchA.get() || loadedLimitSwitchB.get());
            SmartDashboard.putNumber("Intake Target", targetPosition);
            SmartDashboard.putNumber("Intake Current", vertical.getOutputCurrent());
            SmartDashboard.putNumber("Intake Intake Current", intake.getOutputCurrent());
            SmartDashboard.putNumber("Intake Position", encoder.getPosition());
        }
    }
}
