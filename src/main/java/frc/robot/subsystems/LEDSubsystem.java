package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

import java.util.Optional;

public class LEDSubsystem {
    private static LEDSubsystem instance;
    SerialPort serialPort = new SerialPort(9600, SerialPort.Port.kMXP);
    private LEDSubsystem()
    {
        serialPort.writeString("I59\n");
    }

    static public LEDSubsystem getInstance()
    {
        if(instance == null)
        {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    private void setLedString(char color, int strength, int length)
    {
        if(length > 9) {
            length = 9;
        }
        if(strength > 9) {
            strength = 9;
        }
        String s = String.format("%c%d%d", color, strength, length);
        System.out.println(s);
        serialPort.writeString(s);
    }
    public void setBlue(int strength, int length)
    {
        setLedString('B', strength, length);
    }
    public void setRed(int strength, int length)
    {
        setLedString('R', strength, length);
    }
    public void setGreen(int strength, int length)
    {
        setLedString('G', strength, length);
    }
    public void setOrange(int strength, int length)
    {
        setLedString('O', strength, length);
    }
    public void setRainbow(int strength, int length)
    {
        setLedString('U', strength, length);
    }
    public void setWhite(int strength, int length)
    {
        setLedString('I', strength, length);
    }

    public void setTeam()
    {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()) {
            return;
        }
        switch(alliance.get())
        {
            case Red -> setRed(Constants.LEDConstants.MID_STRENGTH,9);
            case Blue -> setBlue(Constants.LEDConstants.MID_STRENGTH,9);
            default -> setWhite(Constants.LEDConstants.MID_STRENGTH, 0);
        }
    }
}
