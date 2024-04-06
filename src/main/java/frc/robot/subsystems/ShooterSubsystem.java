// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.DigitalInputs.SHOOTER_ENCODER);
    PIDController verticalController = new PIDController(3, 0.001, 0);
    double targetPosition = 0;
    private ShooterSubsystem() {
        intake.setSmartCurrentLimit(20);
        intake.setSecondaryCurrentLimit(20);
        amp_extender.setSmartCurrentLimit(20);
        amp_extender.setSecondaryCurrentLimit(20);
        targetPosition = Constants.PositionConstants.Shooter.LAUNCH_SPEAKER;
        verticalController.setSetpoint(targetPosition);
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
            case MOTOR_FORWARD -> intake.set(0.9);
            case MOTOR_BACKWARD -> intake.set(-0.9);
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
                shooterB.set(-1.0 * speed);
                double vel = shooterB.getEncoder().getVelocity();
                LEDSubsystem.getInstance().setGreen(Constants.LEDConstants.MAX_STRENGTH, (int)((Math.abs(vel)/ 6000) * 10));
                if(Math.abs(shooterB.getEncoder().getVelocity()) > (4000 * speed)
                        && LimeLightSensor.getInstance().getSpeakerTargetHorizontalOffset() < Constants.PositionConstants.MIN_POSITION_THRESHOLD)
                {
                    shooterA.set(-1.0 * speed);
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
        targetPosition = MathUtil.clamp(targetPos, Constants.PositionConstants.Shooter.MINIMUM, Constants.PositionConstants.Shooter.MAXIMUM);
        verticalController.setSetpoint(targetPos);
        //System.out.println(power);
    }
    public void setExtenderPower(MotorDirection dir)
    {
        switch (dir)
        {
            case MOTOR_BACKWARD -> amp_extender.set(-0.55);
            case MOTOR_STOP -> amp_extender.set(0);
            case MOTOR_FORWARD -> amp_extender.set(0.55);
        }
    }
    public void setVertical(double moveVal)
    {
        if(Math.abs(moveVal) < 0.2) {
            moveVal = 0;
        }
        setTargetPosition(targetPosition + (moveVal * 0.004));
    }
    public double findShooterPos(){
        return encoder.getAbsolutePosition();
    }
    
    @Override
    public void periodic()
    {
        handleVerticalArmUpdates();
//        double vertP =  SmartDashboard.getNumber("Shooter P", verticalController.getP());
//        if(vertP != verticalController.getP())
//        {
//            verticalController.setP(vertP);
//        }
        if(Constants.DEBUG_INFO) {
            SmartDashboard.putNumber("Shooter Speed", shooterA.getEncoder().getVelocity());
            SmartDashboard.putNumber("Shooter Target", targetPosition);
            SmartDashboard.putNumber("Shooter Current", vertical.getOutputCurrent());
            SmartDashboard.putNumber("Shooter Intake Current", intake.getOutputCurrent());
        }
        SmartDashboard.putNumber("Shooter Position", encoder.getAbsolutePosition());
    }

    private void handleVerticalArmUpdates()
    {
        if(verticalController.getSetpoint() < 0.1 || encoder.getAbsolutePosition() < 0.1)
        {
            return;
        }
        double power = -verticalController.calculate(encoder.getAbsolutePosition());
        power = MathUtil.clamp(power, -0.30, 0.30);
        vertical.set(power);
    }
}