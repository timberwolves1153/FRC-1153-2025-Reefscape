package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.google.flatbuffers.Constants;

import edu.wpi.first.units.measure.Distance;
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
    private LEDPattern baseCoralPattern;
    private LEDPattern baseAlgaePattern;
    private LEDPattern baseAutoPattern;
    private LEDPattern baseScorePattern;
    private LEDPattern baseCollectPattern;
    private LEDPattern coralPattern;
    private LEDPattern algaePattern;
    private LEDPattern autoPattern;
    private LEDPattern scorePattern;
    private LEDPattern collectPattern;
    private LEDPattern scrollAutoPattern;
    private Distance LEDDensity;

    public LED() {

        LEDDensity = Meters.of(0/1); //chanhe tjis to the actual density of the LEDs
        led = new AddressableLED(0);

        ledBuffer = new AddressableLEDBuffer(60);//CHANGES        
        led.setLength(60);//CHANGES

        baseAlgaePattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kTeal, Color.kDarkTurquoise);
        algaePattern = baseAlgaePattern.scrollAtRelativeSpeed(Percent.per(Second).of(25));

        baseCoralPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kWhite, Color.kWhiteSmoke);
        coralPattern = baseCoralPattern.scrollAtRelativeSpeed(Percent.per(Second).of(25));

        baseScorePattern = LEDPattern.rainbow(255, 128);
        scorePattern = baseScorePattern.scrollAtRelativeSpeed(Percent.per(Second).of(25));

        baseCollectPattern = LEDPattern.rainbow(225, 128);
        collectPattern = baseCollectPattern.scrollAtRelativeSpeed(Percent.per(Second).of(25));

        baseAutoPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kOrangeRed, Color.kDarkBlue);
        scrollAutoPattern = baseAutoPattern.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        autoPattern = scrollAutoPattern.blink(Seconds.of(0.3), Seconds.of(0.1));





        
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

    public Command runScorePattern(){
        return run(() -> scorePattern.applyTo(ledBuffer));
    }

    public Command runCollectPattern(){
        return run(() -> collectPattern.applyTo(ledBuffer));
    }



}
