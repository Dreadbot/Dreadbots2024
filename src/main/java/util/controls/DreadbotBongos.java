package util.controls;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is written for programming with Donkey Kong bongos
 * Picture : https://external-content.duckduckgo.com/iu/?u=https%3A%2F%2Ftse2.mm.bing.net%2Fth%3Fid%3DOIP.jNnVXNXn7iaGfG8wWdlvDwHaE5%26pid%3DApi&f=1
 * This class makes it easier to program and implement DK bongos into your project
 */
public class DreadbotBongos {
    private Joystick bongos;

    public DreadbotBongos(int joystickPort){
        this.bongos = new Joystick(joystickPort);
    }

    /**
     * Gets the button input from the bottom left button on the bongo
     * @return Whether the bottom left button is pressed
     */
    public boolean isLeftBingPressed(){
        return bongos.getRawButtonPressed(3);
    }

    /**
     * Gets the button input from the top left button on the bongo
     * @return Whether the top left button is pressed
     */
    public boolean isLeftBongPressed(){
        return bongos.getRawButtonPressed(4);
    }

    /**
     * Gets the button input from the bottom right button on the bongo
     * @return Whether the bottom right button is pressed
     */
    public boolean isRightBingPressed(){
        return bongos.getRawButtonPressed(2);
    }

    /**
     * Gets the button input from the top right button on the bongo
     * @return Whether the top right button is pressed
     */
    public boolean isRightBongPressed(){
        return bongos.getRawButtonPressed(1);
    }

    /**
     * Gets the button input from the start button on the bongo
     * @return Whether the start button is pressed
     */
    public boolean isStartButtonPressed(){
        return bongos.getRawButtonPressed(10);
    }

    /**
     * Gets the Mic input and filters it so it starts at 0 and go's up
     * @return How much noise is detected by the mic
     */
    public double getClapMicInputFiltered(){
        if(bongos.getRawAxis(4) == -1){
            return 0;
        }
        double filteredInput = (bongos.getRawAxis(4) + 1)/2;
        if (filteredInput <= 0.05)
            return 0;
        else
            return filteredInput;
    }

    /**
     * Gets the input from the mic unfiltered
     * @return How much noise is detected by the mic
     */
    public double getClapMicInput(){
        return bongos.getRawAxis(4);
    }
}
