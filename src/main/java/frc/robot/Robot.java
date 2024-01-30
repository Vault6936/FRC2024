// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.GyroCalibrateCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.webdashboard.DashboardLayout;
import frc.robot.webdashboard.WebdashboardServer;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    public static Timer timer = new Timer();
    public static RobotContainer robotContainer;
    Limelight limelight = Limelight.getInstance();
    PowerDistribution pdh = new PowerDistribution(Constants.CANIds.PDH_ID, PowerDistribution.ModuleType.kRev);
    private Command autonomousCommand;

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        timer.start();
        WebdashboardServer.getInstance(5800); // Initialize the websocket server
        CommandScheduler.getInstance().schedule(new GyroCalibrateCommand(1000));
    }


    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        limelight.update();
        //DashboardLayout.setNodeValue("example pdh channel", pdh.getCurrent(0));
        CommandScheduler.getInstance().run();
    }


    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }


    @Override
    public void disabledPeriodic() {
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        DriveSubsystem.getInstance().resetGyro();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }


    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        DriveSubsystem.getInstance().resetGyro();
        DriveSubsystem.getInstance().boot();
        DriveSubsystem.getInstance().chassis.boot();
        GlobalVariables.pose = new Pose2d();
    }


    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }


    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }


    /**
     * This method is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }


    /**
     * This method is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
