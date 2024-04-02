// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase
{
    private static ShooterSubsystem instance;
    CANSparkMax shooterA = new CANSparkMax(Constants.CANIds.SHOOTER_MOTOR_A, CANSparkMax.MotorType.kBrushless);
    CANSparkMax shooterB = new CANSparkMax(Constants.CANIds.SHOOTER_MOTOR_B, CANSparkMax.MotorType.kBrushless);
    CANSparkMax intake = new CANSparkMax(Constants.CANIds.SHOOTER_MOTOR_INTAKE, CANSparkMax.MotorType.kBrushless);
    CANSparkMax vertical = new CANSparkMax(Constants.CANIds.SHOOTER_MOTOR_ELEVATION, CANSparkMax.MotorType.kBrushless);
    CANSparkMax amp_extender = new CANSparkMax(Constants.CANIds.SHOOTER_AMP_EXTENDER, CANSparkMax.MotorType.kBrushed);
    RelativeEncoder encoder;
    SparkPIDController verticalController;
    double targetPosition = 0;
    private ShooterSubsystem() {
        verticalController = vertical.getPIDController();
        verticalController.setP(0.2);
        verticalController.setOutputRange(-0.4,0.4);
        intake.setSmartCurrentLimit(20);
        intake.setSecondaryCurrentLimit(20);
        amp_extender.setSmartCurrentLimit(20);
        amp_extender.setSecondaryCurrentLimit(15);
        encoder = vertical.getEncoder();
        encoder.setPosition(0.0);
    }

    public static ShooterSubsystem getInstance()
    {
        if(instance == null)
        {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    public void setIntakeMotor(MotorDirection dir)
    {
        switch (dir) {
            case MOTOR_FORWARD -> intake.set(0.7);
            case MOTOR_BACKWARD -> intake.set(-0.7);
            case MOTOR_STOP -> intake.set(0);
        }
    }

    public void setShooterMotors(MotorDirection dir, double speed) {
        switch (dir) {
            case MOTOR_FORWARD -> {
                shooterA.set(0.2);
                shooterB.set(0.2);
            }
            case MOTOR_BACKWARD -> {
                shooterB.set(-0.8 * speed);
                double vel = shooterB.getEncoder().getVelocity();
                LEDSubsystem.getInstance().setGreen(Constants.LEDConstants.MAX_STRENGTH, (int)((vel/ 6000) * 10));
                if(Math.abs(shooterB.getEncoder().getVelocity()) > (3000 * speed))
                {
                    shooterA.set(-0.8 * speed);
                }
            }
            case MOTOR_STOP -> {
                LEDSubsystem.getInstance().setTeam();
                shooterA.set(0);
                shooterB.set(0);
            }
        }
    }

    public void setTargetPosition(double targetPos){
        targetPosition = MathUtil.clamp(targetPos, -24.1, -0.1);
        verticalController.setReference(targetPosition, CANSparkBase.ControlType.kPosition);
    }
    public void setExtenderPower(MotorDirection dir)
    {
        switch (dir)
        {
            case MOTOR_BACKWARD -> amp_extender.set(-0.9);
            case MOTOR_STOP -> amp_extender.set(0);
            case MOTOR_FORWARD -> amp_extender.set(0.9);
        }
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
//        double vertP =  SmartDashboard.getNumber("Shooter P", verticalController.getP());
//        if(vertP != verticalController.getP())
//        {
//            verticalController.setP(vertP);
//        }
//        SmartDashboard.putNumber("Shooter Speed", shooterA.getEncoder().getVelocity());
//        SmartDashboard.putNumber("Shooter Position", encoder.getPosition());
//        SmartDashboard.putNumber("Shooter Target", targetPosition);
//        SmartDashboard.putNumber("Shooter Current", vertical.getOutputCurrent());
//        SmartDashboard.putNumber("Shooter Intake Current", intake.getOutputCurrent());
    }
}