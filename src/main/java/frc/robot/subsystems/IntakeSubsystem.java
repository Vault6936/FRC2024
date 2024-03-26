//package frc.robot.subsystems;
//
//import com.revrobotics.CANSparkMax;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
//
//public class IntakeSubsystem extends SubsystemBase {
//
//    CANSparkMax driveMotor;
//    CANSparkMax turnMotor;
//
//    DutyCycleEncoder encoder;
//
//    PIDController controller;
//
//    public IntakeSubsystem() {
//        driveMotor = new CANSparkMax(Constants.CANIds.intakeDriveMotor, CANSparkMax.MotorType.kBrushless);
//        turnMotor = new CANSparkMax(Constants.CANIds.intakeTurnMotor, CANSparkMax.MotorType.kBrushless);
//        encoder = new DutyCycleEncoder(Constants.EncoderPorts.intakePort);
//        controller = new PIDController(0.1, 0, 0);
//    }
//
//    private double getFeedforward() {
//        return Math.cos(encoder.getAbsolutePosition() * Math.PI * 2);
//    }
//
//    public void setAngle(double angle) {
//        angle = angle % 360;
//        turnMotor.set(getFeedforward() + controller.calculate(encoder.getAbsolutePosition() * Math.PI * 2, angle));
//    }
//
//    @Override
//    public void periodic() {
//
//    }
//}
