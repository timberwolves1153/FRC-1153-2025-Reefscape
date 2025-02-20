package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Seconds;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private LEDPattern coralPattern;
    private LEDPattern algaePattern;
    private LEDPattern baseAutoPattern;
    private LEDPattern autoPattern;

    public LED() {
        led = new AddressableLED(0);

        ledBuffer = new AddressableLEDBuffer(60);//CHANGES        
        led.setLength(60);//CHANGES

        algaePattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kTeal, Color.kDarkTurquoise);
        coralPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kWhite, Color.kWhiteSmoke);

        baseAutoPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kOrangeRed, Color.kDarkBlue);
        autoPattern = baseAutoPattern.blink(Seconds.of(0.3), Seconds.of(0.1));



        
        led.start();
    }

@Override
    public void periodic() {
    led.setData(ledBuffer);
    }

    public Command runCoralPattern(){
        return run(() -> coralPattern.applyTo(ledBuffer));
    }

    public Command runAlgaePattern(){
        return run(() -> algaePattern.applyTo(ledBuffer));
    }

    public Command runAutoPattern(){
        return run(() -> autoPattern.applyTo(ledBuffer));
    }



}
