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
    public void setClimbCurrentPos(double currentVal)
    {
        climberLeft.setSelectedSensorPosition(currentVal);
        climberRight.setSelectedSensorPosition(currentVal);
    }
    public void setClimbPower(double left, double right, ClimbMotors motors)
    {
        switch(motors) {
            case BOTH: {
                climberLeft.set(left);
                climberRight.set(right);
                break;
            }
            case RIGHT:
            {
                climberRight.set(right);
                break;
            }
            case LEFT:
            {
                climberLeft.set(left);
                break;
            }
        }
    }

    public void setClimbLeft(double left)
    {
        climberLeft.set(left);
    }

    public void setClimbRight(double power)
    {
        climberRight.set(power);
    }
    public double getRightVelocity()
    {
        return climberRight.getSelectedSensorVelocity();
    }

    public double getLeftVelocity()
    {
        return climberLeft.getSelectedSensorVelocity();
    }

    @Override
    public void periodic(){
        if(Constants.DEBUG_INFO) {

        }
        SmartDashboard.putNumber("ClimbLeftPos: ", climberLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("ClimbRightPos: ", climberRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("ClimbRightVel: ", climberRight.getSelectedSensorVelocity());
    }
}
