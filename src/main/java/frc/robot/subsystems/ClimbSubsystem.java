package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
@SuppressWarnings("removal")
public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem instance;
    com.ctre.phoenix.motorcontrol.can.WPI_TalonFX climberLeft = new com.ctre.phoenix.motorcontrol.can.WPI_TalonFX(Constants.CANIds.CLIMBER_LEFT);
    com.ctre.phoenix.motorcontrol.can.WPI_TalonFX climberRight = new com.ctre.phoenix.motorcontrol.can.WPI_TalonFX(Constants.CANIds.CLIMBER_RIGHT);
    double leftPosition = 0;
    double rightPosition = 0;
    private ClimbSubsystem() {
        climberLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,20,20,.1));
        climberRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,20,20,.1));
//        climberRight.config_kP(0,.01);
//        climberLeft.config_kP(0,.01);
//
//        climberRight.setSelectedSensorPosition(0);
//        climberLeft.setSelectedSensorPosition(0);
    }

    public static ClimbSubsystem getInstance(){
        if (instance == null){
            instance = new ClimbSubsystem();
        }
        return instance;
    }

    public void setClimbPos(double leftY, double rightY)
    {
        leftPosition = MathUtil.clamp(leftPosition + leftY,-100,100);
        rightPosition = MathUtil.clamp(rightPosition + rightY,-100,100);
        climberRight.set(ControlMode.Position, rightPosition);
        climberLeft.set(ControlMode.Position, leftPosition);
    }

    public void setClimbPower(double left, double right)
    {
        climberLeft.set(left);
        climberRight.set(right);
    }
    @Override
    public void periodic(){
        //SmartDashboard.putNumber("leftPos: ", climberLeft.getSelectedSensorPosition());
        //SmartDashboard.putNumber("RightPos: ", climberRight.getSelectedSensorPosition());
    }
}
