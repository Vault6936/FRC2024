// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeToShooterSubsystem extends SubsystemBase
{
    CANSparkMax intake = new CANSparkMax(Constants.CANIds.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
    CANSparkMax vertical = new CANSparkMax(Constants.CANIds.INTAKE_VERTICAL, CANSparkMax.MotorType.kBrushless);
    DigitalInput loadedLimitSwitch = new DigitalInput(Constants.DigitalInputs.INTAKE_HOLD);
    SparkPIDController verticalController;
    RelativeEncoder encoder;

    /** Creates a new ExampleSubsystem. */
    public IntakeToShooterSubsystem() {
        encoder = vertical.getEncoder();
        encoder.setPosition(0.0);
        intake.setSecondaryCurrentLimit(20);
        intake.setSmartCurrentLimit(20);
        verticalController = vertical.getPIDController();
        verticalController.setP(0.034 + .001);
        verticalController.setOutputRange(-0.4,0.4);
    }
    double targetPosition = 0;
    public void intake(MotorDirection dir)
    {
        switch (dir) {
            case MOTOR_FORWARD -> intake.set(1.0);
            case MOTOR_BACKWARD -> {
                intake.set(-0.4);
                break;
            }
            case MOTOR_STOP -> intake.set(0);
        }
    }
    public void setTargetPosition(double TargetPos){
        targetPosition = MathUtil.clamp(TargetPos, 0, 36.0);
        verticalController.setReference(targetPosition, CANSparkBase.ControlType.kPosition);
    }
    public void setVertical(double val) {
        if (Math.abs(val) < 0.2) {
            val = 0;
        }
        val = MathUtil.clamp(val, -0.4, 0.4);
        setTargetPosition(targetPosition + (val * 0.36));

    }

    public double findIntakePos(){
        return encoder.getPosition();

    }



    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Intake Target", targetPosition);
        SmartDashboard.putNumber("Intake Current", vertical.getOutputCurrent());
        SmartDashboard.putNumber("Intake Intake Current", intake.getOutputCurrent());
        SmartDashboard.putNumber("Intake Position", encoder.getPosition());
        // This method will be called once per scheduler run
    }
}
