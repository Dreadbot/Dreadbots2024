package util.controls;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A Wrapper Class to make coding with a Logetech Tractor
 * controller easier and simplier
 */
@SuppressWarnings("unused")
public class DreadbotTractorController {
    private final Joystick tractorController;
    /**
     * Creates a DreadbotTractorController object with a port and 
     * passing it into a WPI lib joystick
     * 
     * @param joystickPort The port of the joystick on Driver Station
     */
    public DreadbotTractorController(int joystickPort){
        this.tractorController = new Joystick(joystickPort);
    }

    /**
     * Returns the reading of moving the joystick left to right on mode 1
     * @return The left to right movement in mode 1
     */
    public double getXAxis(){
        return tractorController.getRawAxis(0);
    }

    /**
     * Returns the reading of moving the joystick up to down on mode 1
     * @return The up to down movement in mode 1
     */
    public double getYAxis(){
        return tractorController.getRawAxis(1);
    }

    /**
     * Returns the reading of twisting the joystick left to right on mode 1
     * @return The twisting left to right in mode 1
     */
    public double getZAxis(){
        return tractorController.getRawAxis(2);
    }

    /**
     * Returns the reading of moving the joystick left to right on mode 2
     * @return The left to right movement in mode 2
     */
    public double getXRotation(){
        return tractorController.getRawAxis(3);
    }

    /**
     * Returns the reading of moving the joystick up to down on mode 2
     * @return The up to down movement in mode 2
     */
    public double getYRotation(){
        return tractorController.getRawAxis(4);
    }

    /**
     * Returns the reading of twisting the joystick left to right on mode 2
     * @return The twisting left to right in mode 2
     */
    public double getZRotation(){
        return tractorController.getRawAxis(5);
    }

    /**
     * Gets the state of the one button on the Tractor controller
     * @return The status of the one button
     */
    public boolean isButton1Pressed(){
        return tractorController.getRawButton(1);
    }

    /**
     * Gets the state of the two button on the Tractor controller
     * @return The status of the two button
     */
    public boolean isButton2Pressed(){
        return tractorController.getRawButton(2);
    }

    /**
     * Gets the state of the three button on the Tractor controller
     * @return The status of the three button
     */
    public boolean isButton3Pressed(){
        return tractorController.getRawButton(3);
    }

    /**
     * Gets the state of the four button on the Tractor controller
     * @return The status of the four button
     */
    public boolean isButton4Pressed(){
        return tractorController.getRawButton(4);
    }

    /**
     * Gets the state of the five button on the Tractor controller
     * @return The status of the five button
     */
    public boolean isButton5Pressed(){
        return tractorController.getRawButton(5);
    }

    /**
     * Gets the state of the six button on the Tractor controller
     * @return The status of the six button
     */
    public boolean isButton6Pressed(){
        return tractorController.getRawButton(6);
    }

    /**
     * Gets the state of the seven button on the Tractor controller
     * @return The status of the seven button
     */
    public boolean isButton7Pressed(){
        return tractorController.getRawButton(7);
    }

    /**
     * Gets the state of the eight button on the Tractor controller
     * @return The status of the eight button
     */
    public boolean isButton8Pressed(){
        return tractorController.getRawButton(8);
    }

    /**
     * Gets the state of the nine button on the Tractor controller
     * @return The status of the nine button
     */
    public boolean isButton9Pressed(){
        return tractorController.getRawButton(9);
    }

    /**
     * Gets the state of the ten button on the Tractor controller
     * @return The status of the ten button
     */
    public boolean isButton10Pressed(){
        return tractorController.getRawButton(10);
    }

    /**
     * Gets the state of the eleven button on the Tractor controller
     * @return The status of the eleven button
     */
    public boolean isButton11Pressed(){
        return tractorController.getRawButton(11);
    }

    /**
     * Gets the state of the twelve button on the Tractor controller
     * @return The status of the twelve button
     */
    public boolean isButton12Pressed(){
        return tractorController.getRawButton(12);
    }

    /**
     * Gets the state of the thirteen button on the Tractor controller
     * @return The status of the thirteen button
     */
    public boolean isButton13Pressed(){
        return tractorController.getRawButton(13);
    }

    /**
     * Gets the state of the fourteen button on the Tractor controller
     * @return The status of the fourteen button
     */
    public boolean isButton14Pressed(){
        return tractorController.getRawButton(14);
    }

    /**
     * Gets the state of the fifteen button on the Tractor controller
     * @return The status of the fifteen button
     */
    public boolean isButton15Pressed(){
        return tractorController.getRawButton(15);
    }

    /**
     * Gets the state of the sixteen button on the Tractor controller
     * @return The status of the sixteen button
     */
    public boolean isButton16Pressed(){
        return tractorController.getRawButton(16);
    }

    /**
     * Gets the state of the seventeen switch on the Tractor controller
     * @return The status of the seventeen switch
     */
    public boolean isSwitch17Pressed(){
        return tractorController.getRawButton(17);
    }

    /**
     * Gets the state of the eighteen switch on the Tractor controller
     * @return The status of the eighteen switch
     */
    public boolean isSwitch18Pressed(){
        return tractorController.getRawButton(18);
    }

    /**
     * Gets the state of the nineteen switch on the Tractor controller
     * @return The status of the nineteen switch
     */
    public boolean isSwitch19Pressed(){
        return tractorController.getRawButton(19);
    }

    /**
     * Gets the state of the twenty switch on the Tractor controller
     * @return The status of the twenty switch
     */
    public boolean isSwitch20Pressed(){
        return tractorController.getRawButton(20);
    }

    /**
     * Gets the state of the bar(21) on the Tractor controller
     * @return The status of the bar
     */
    public boolean isBarPressed(){
        return tractorController.getRawButton(21);
    }

    /**
     * Gets the state of the twenty-two button on the Tractor controller
     * @return The status of the twenty-two button
     */
    public boolean isButton22Pressed(){
        return tractorController.getRawButton(22);
    }

    /**
     * Gets the state of the twenty-three button on the Tractor controller
     * @return The status of the twenty-three button
     */
    public boolean isButton23Pressed(){
        return tractorController.getRawButton(23);
    }

    /**
     * Gets the state of the twenty-four button on the Tractor controller
     * @return The status of the twenty-four button
     */
    public boolean isButton24Pressed(){
        return tractorController.getRawButton(24);
    }

    /**
     * Gets the state of the Joystick button on the Tractor controller
     * @return The status of the Joystick button
     */
    public boolean isJoystickButtonPressed(){
        return tractorController.getRawButton(28);
    }

    /**
     * Gets the state of the mouse wheel scrolling up on the Tractor controller
     * @return The status of the mouse wheel scrolling up
     */
    public boolean isMouseWheelScrollingUp(){
        return tractorController.getRawButton(26);
    }

    /**
     * Gets the state of the mouse wheel scrolling down on the Tractor controller
     * @return The status of the mouse wheel scrolling down
     */
    public boolean isMouseWheelScrollingDown(){
        return tractorController.getRawButton(27);
    }

    /**
     * Gets the state of the mouse wheel being clicked on the Tractor controller
     * @return The status of the mouse wheel scrolling being clicked
     */
    public boolean isMouseWheelClicked(){
        return tractorController.getRawButton(25);
    }

    /**
     * Gets the WPI Lib joystick object inside this class
     *
     * @return WPI Lib joystick object
     */
    public Joystick getWPIJoystick(){
        return tractorController;
    }

    /**
     * Uses the original getRawButtonMethod from WPI
     * @param button The ID of the button
     * @return Whether the button is pressed
     */
    public boolean getRawButton(int button){
        return tractorController.getRawButton(button);
    }

    /**
     * Uses the original getRawAxis to get the value of an axis
     * @param axis The ID of the axis
     * @return The value of the axis
     */
    public double getRawAxis(int axis){
        return tractorController.getRawAxis(axis);
    }

    /**
     * Returns Whether the controller is connected or not
     * @return Whether the controller is connected or not
     */
    public boolean isConnected(){
        return tractorController.isConnected();
    }

    /**
     * Gets the name of the controller and adds that it is a tractor controller
     * @return Name of the Tractor controller
     */
    public String getName(){
        return  "Tractor Controller " + tractorController.getName();
    }
}
