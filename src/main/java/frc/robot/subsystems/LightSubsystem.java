package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.LightConstants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import frc.robot.RobotConstants.PortConstants;

public class LightSubsystem extends SubsystemBase {

    //public final String setIdleMode = null;
    CANdle candle;
    CANdleConfiguration candleConfig;
    
    public LightSubsystem(){

        try{
            candle = new CANdle(PortConstants.CAN.kLightPort);
            candleConfig = new CANdleConfiguration();
            candleConfig.stripType = LEDStripType.RGB;
            candleConfig.statusLedOffWhenActive = true;
            candleConfig.disableWhenLOS = false;
            candleConfig.v5Enabled = true;
            candleConfig.vBatOutputMode = VBatOutputMode.On;
            candleConfig.brightnessScalar = LightConstants.kLightBrightness;
            candle.configAllSettings(candleConfig);
        }
        catch (Exception e){
            System.out.println("Error: " + e);
        }
    }

    /**
     *
     * @apiNote Used to set a premade animation from this list: ColorFlowAnimation,
     * @apiNote FireAnimation, LarsonAnimation, RainbowAnimation, RgbFadeAnimation,
     * @apiNote SingleFadeAnimation, StrobeAnimation, TwinkleAnimation, TwinkleOffAnimation 
     * @param animation The animation being set
     */
    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }

    /**
     *
     * @apiNote Used to set a premade animation from this list: ColorFlowAnimation,
     * @apiNote FireAnimation, LarsonAnimation, RainbowAnimation, RgbFadeAnimation,
     * @apiNote SingleFadeAnimation, StrobeAnimation, TwinkleAnimation, TwinkleOffAnimation 
     * @param animation The animation being set
     * @param slot The animation slot being used
     */
    public void setAnimation(Animation animation, int slot){
        candle.animate(animation, slot);
    }

    public void stopAnimation(int slot){
        candle.clearAnimation(slot);
    }

    /**
     * 
     * @apiNote Used to set the specific colors for the whole LED strip
     * @param red The amount of red from 0-255
     * @param green The amount of green from 0-255
     * @param blue The amount of blue from 0-255
     */
    public void setColor(int red, int green, int blue){
        candle.setLEDs(red, green, blue);
    }

    /**
     * 
     * @apiNote Used to set specific colors in specific sections of the LED strip
     * @param red The amount of red from 0-255
     * @param green The amount of green from 0-255
     * @param blue The amount of blue from 0-255
     * @param white The amount of white from 0-255
     * @param startindex The starting LED for this setColor method
     * @param count The number of LEDs to apply this setColor method to
     */
    public void setColor(int red, int green, int blue, int white, int startindex, int count){
        candle.setLEDs(red, green, blue, white, startindex, count);
    }

    public double getCurrent() {
        return candle.getCurrent();
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Light Current", getCurrent());
    }
}