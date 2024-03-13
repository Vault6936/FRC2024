// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase
{
    CANSparkMax shooterA = new CANSparkMax(Constants.CANIds.SHOOTER_MOTOR_A, CANSparkMax.MotorType.kBrushless);
    CANSparkMax shooterB = new CANSparkMax(Constants.CANIds.SHOOTER_MOTOR_B, CANSparkMax.MotorType.kBrushless);
    CANSparkMax intake = new CANSparkMax(Constants.CANIds.SHOOTER_MOTOR_INTAKE, CANSparkMax.MotorType.kBrushless);
    CANSparkMax vertical = new CANSparkMax(Constants.CANIds.SHOOTER_MOTOR_ELEVATION, CANSparkMax.MotorType.kBrushless);
    RelativeEncoder encoder;
    SparkPIDController verticalController;
    double targetPosition = 0;

    /** Creates a new ExampleSubsystem. */
    public ShooterSubsystem() {
        verticalController = vertical.getPIDController();
        verticalController.setP(0.2);
        verticalController.setOutputRange(-0.4,0.4);
        intake.setSmartCurrentLimit(20);
        intake.setSecondaryCurrentLimit(20);
        encoder = vertical.getEncoder();
        encoder.setPosition(0.0);
    }

    public void setIntakeMotor(MotorDirection dir)
    {
        switch (dir) {
            case MOTOR_FORWARD -> intake.set(0.7);
            case MOTOR_BACKWARD -> intake.set(-0.7);
            case MOTOR_STOP -> intake.set(0);
        }
    }

    public void setShooterMotors(MotorDirection dir) {
        switch (dir) {
            case MOTOR_FORWARD -> {
                shooterA.set(0.2);
                shooterB.set(0.2);

            }
            case MOTOR_BACKWARD -> {
                shooterB.set(-0.35);

                //if(Math.abs(shooterB.getEncoder().getVelocity()) > (5000)) {
                    shooterA.set(-0.35);
                //}
            }
            case MOTOR_STOP -> {
                shooterA.set(0);
                shooterB.set(0);
            }
        }
    }

    public void setTargetPosition(double targetPos){
        targetPosition = MathUtil.clamp(targetPos, -24.1, -0.1);
        verticalController.setReference(targetPosition, CANSparkBase.ControlType.kPosition);
}
    public void setVertical(double moveVal)
    {
        if(Math.abs(moveVal) < 0.2) {
            moveVal = 0;
        }
        setTargetPosition(targetPosition + (moveVal * 0.12));
    }
    public double findShooterPos(){
        return encoder.getPosition()                                                                  ;
    }
    
    @Override
    public void periodic()
    {
        double vertP =  SmartDashboard.getNumber("Shooter P", verticalController.getP());
        if(vertP != verticalController.getP())
        {
            verticalController.setP(vertP);
        }
        SmartDashboard.putNumber("Shooter Speed", shooterA.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Position", encoder.getPosition());
        SmartDashboard.putNumber("Shooter Target", targetPosition);
        SmartDashboard.putNumber("Shooter Current", vertical.getOutputCurrent());
        SmartDashboard.putNumber("Shooter Intake Current", intake.getOutputCurrent());
        // This method will be called once per scheduler run
    }
}
