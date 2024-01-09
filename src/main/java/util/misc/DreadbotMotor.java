package util.misc;
import java.util.logging.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

@SuppressWarnings({"unused"})
public class DreadbotMotor{
    private final CANSparkMax motor;
    private final RelativeEncoder motorEncoder;
    private final SparkMaxPIDController motorPIDController;
    private final String name;
    private boolean isDisabled = false;
    private Logger logger = Logger.getLogger(Robot.class.getName());

    /**
     * Constructs a DreadbotMotor object by taking in a motor and a name of the motor
     *
     * @param motor The motor that is being used for Dreadbot Motor
     * @param name The name of the motor
     */
    public DreadbotMotor(CANSparkMax motor, String name){
        this.motor = motor;
        this.motorEncoder = motor.getEncoder();
        this.motorPIDController = motor.getPIDController();
        this.name = name + " motor";
    }

    /**
     * Restores the factory defaults of the motor controller until the motor reboots
     *
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError restoreFactoryDefaults(){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motor.restoreFactoryDefaults();
        } catch (RuntimeException ignored) {
            disable();
            printError("restoreFactoryDefaults");
            return REVLibError.kError;
        }
    }

    /**
     * Sets the idle mode of the motor
     *
     * @param mode The idle mode that is being set on the Motor (coast or brake)
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     * */
    public REVLibError setIdleMode(IdleMode mode){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motor.setIdleMode(mode);
        } catch (RuntimeException ignored) {
            disable();
            printError("setIdleMode");
            return REVLibError.kError;
        }
    }

    /**
     * Inverts the speed of the motor
     *
     * @param isInverted State of inversion, set to true for inverted
     */
    public void setInverted(boolean isInverted){
        if(isDisabled()) return;
        try{
            motor.setInverted(isInverted);
        } catch (RuntimeException ignored) {
            disable();
            printError("setInverted");
        }
    }

    /**
     * Sets conversion factor for the motor encoder to get to your desired units for position
     * to give a position
     *
     * @param factor Conversion factor to get to your native units
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setPositionConversionFactor(Double factor){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorEncoder.setPositionConversionFactor(factor);
        } catch (RuntimeException ignored) {
            disable();
            printError("setPositionConversionFactor");
            return REVLibError.kError;
        }
    }

    /**
     * Sets the conversion factor for the motor encoder to get to your desired units for velocity
     *
     * @param factor The conversion factor to get to the needed units for velocity
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setVelocityConversionFactor(double factor){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorEncoder.setVelocityConversionFactor(factor);
        } catch (RuntimeException ignored) {
            disable();
            printError("setVelocityConversionFactor");
            return REVLibError.kError;
        }
    }

    /**
     * Gets the velocity of the motor
     *
     * @return RPM of the motor, and multiplied by the Velocity
     * conversion factor to get to your desired units if set
     */
    public double getVelocity(){
        if(isDisabled()) return 0;
        try{
            return motorEncoder.getVelocity();
        } catch (RuntimeException ignored) {
            disable();
            printError("getVelocity");
            return 0;
        }
    }

    /**
     * Sets the voltage output of the motor
     *
     * @param outputVolts The voltage to output
     */
    public void setVoltage(double outputVolts){
        if(isDisabled()) return;
        try{
            motor.setVoltage(outputVolts);
        } catch (RuntimeException ignored) {
            disable();
            printError("setVoltage");
        }
    }

    /**
     * Sets position of the motors encoder
     *
     * @param position The desired position of the motor encoder
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setPosition(double position){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorEncoder.setPosition(position);
        } catch (RuntimeException ignored) {
            disable();
            printError("setPosition");
            return REVLibError.kError;
        }
    }

    /**
     * Sets motor encoder position to 0.0
     *
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError resetEncoder(){
        if(isDisabled()) return REVLibError.kError;
        try{
            return setPosition(0.0d);
        } catch (RuntimeException ignored) {
            disable();
            printError("resetEncoder");
            return REVLibError.kError;
        }
    }

    /**
     * Closes the motor object
     */
    public void close(){
        if(isDisabled()) return;
        try{
            motor.close();
        } catch (IllegalStateException ignored) {
            disable();
            printError("close");
        }
    }

    /**
     * Gets the speed of the motor
     *
     * @return Speed of the motor
     */
    public double get(){
        if(isDisabled()) return 0;
        try{
            return motor.get();
        } catch (RuntimeException ignored) {
            disable();
            printError("get");
            return 0;
        }
    }

    /**
     * Sets the speed of the motor
     *
     * @param speed The speed to be set on the motor
     */
    public void set(double speed){
        if(isDisabled()) return;
        try{
            motor.set(speed);
        } catch (RuntimeException ignored) {
            disable();
            printError("set");
        }
    }

    /**
     * Stops the motor from moving, can be re-enabled by calling set()
     */
    public void stopMotor(){
        if(isDisabled()) return;
        try{
            motor.stopMotor();
        } catch (RuntimeException ignored) {
            disable();
            printError("close");
        }
    }

    /**
     * Sets the reference value of the PID controller based on the control type
     *
     * @param value The value being set based on the control type, should be between -1 and 1
     * @param ctrl control type
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setReference(double value, ControlType ctrl){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorPIDController.setReference(value, ctrl);
        } catch (RuntimeException ignored){
            disable();
            printError("setReference");
            return REVLibError.kError;
        }
    }

    /**
     * Gets the Position of the motor in rotations or the desired units,
     * can be changed with setPositionConversion Factor
     *
     * @return Number of rotations of the motor
     */
    public double getPosition(){
        if(isDisabled()) return 0.0;
        try{
            return motorEncoder.getPosition();
        } catch (RuntimeException ignored) {
            disable();
            printError("getOutputMin");
            return 0.0;
        }
    }

    /**
     * Sets the proportional gain constant for the motors PID controller
     *
     * @param gain The proportional gain
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setP(double gain){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorPIDController.setP(gain);
        } catch (RuntimeException ignored) {
            disable();
            printError("setP");
            return REVLibError.kError;
        }
    }

    /**
     * Get the proportional gain constant of the motors PID controller
     *
     * @return The proportional gain, must be positive
     */
    public double getP(){
        if(isDisabled()) return 0;
        try{
            return motorPIDController.getP();
        } catch (RuntimeException ignored) {
            disable();
            printError("getP");
            return 0;
        }
    }

    /**
     * Sets integral gain constant for the motors PID controller
     *
     * @param gain The integral gain, must be positive
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setI(double gain){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorPIDController.setI(gain);
        } catch (RuntimeException ignored) {
            disable();
            printError("setI");
            return REVLibError.kError;
        }
    }

    /**
     * Gets the integral gain constant of the motors PID controller
     *
     * @return The integral gain
     */
    public double getI(){
        if(isDisabled()) return 0;
        try{
            return motorPIDController.getI();
        } catch (RuntimeException ignored) {
            disable();
            printError("getI");
            return 0;
        }
    }

    /**
     * Sets the derivative gain of the motors PID controller
     *
     * @param gain The derivative gain, must be positive
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setD(double gain){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorPIDController.setD(gain);
        } catch (RuntimeException ignored) {
            disable();
            printError("setD");
            return REVLibError.kError;
        }
    }

    /**
     * Gets the derivative gain of the motors PID controller
     *
     * @return The derivative gain
     */
    public double getD(){
        if(isDisabled()) return 0;
        try{
            return motorPIDController.getD();
        } catch (RuntimeException ignored) {
            disable();
            printError("getD");
            return 0;
        }
    }

    /**
     * Sets the error range for the integral gain for the motors PID controller
     *
     * @param Izone The error range of the integral gain
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setIZone(double Izone){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorPIDController.setIZone(Izone);
        } catch (RuntimeException ignored) {
            disable();
            printError("setIZone");
            return REVLibError.kError;
        }
    }

    /**
     *  Gets the error range of the integral zone of the motors PID controller
     *
     * @return The error range of the integral zone
     */
    public double getIZone(){
        if(isDisabled()) return 0;
        try{
            return motorPIDController.getIZone();
        } catch (RuntimeException ignored) {
            disable();
            printError("getIZone");
            return 0;
        }
    }

    /**
     * Sets the feed forward gain of the motors PID controller
     *
     * @param gain The feed forward gain
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setFF(double gain){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorPIDController.setFF(gain);
        } catch (RuntimeException ignored) {
            disable();
            printError("setFF");
            return REVLibError.kError;
        }
    }

    /**
     * Gets the feed forward gain from the motors PID controller
     *
     * @return The feed forward gain
     */
    public double getFF(){
        if(isDisabled()) return 0;
        try{
            return motorPIDController.getFF();
        } catch (RuntimeException ignored) {
            disable();
            printError("getFF");
            return 0;
        }
    }

    /**
     * Sets the min and max output for the PID controllers closed loop mode
     *
     * @param min The minimum output
     * @param max The maximum output
     * @return RevLibError.kOk if successful, RevLibError.kError if something went wrong
     */
    public REVLibError setOutputRange(double min, double max){
        if(isDisabled()) return REVLibError.kError;
        try{
            return motorPIDController.setOutputRange(min,max);
        } catch (RuntimeException ignored) {
            disable();
            printError("setOutputRange");
            return REVLibError.kError;
        }
    }

    /**
     * Gets the maximum output of the motors PID controller
     *
     * @return The max output of the PID controller
     */
    public double getOutputMax(){
        if(isDisabled()) return 0.0;
        try{
            return motorPIDController.getOutputMax();
        } catch (RuntimeException ignored) {
            disable();
            printError("getOutputMax");
            return 0.0;
        }
    }

    /**
     * Gets the minimum output of the motors PID controller
     *
     * @return The min output of the PID controller
     */
    public double getOutputMin(){
        if(isDisabled()) return 0.0;
        try{
            return motorPIDController.getOutputMin();
        } catch (RuntimeException ignored) {
            disable();
            printError("getOutputMin");
            return 0.0;
        }
    }

    /**
     * Puts out the PID values of the motor to smart dashboard to be able to set the values without
     * restarting the bot
     */
    public void PIDTuner(){
        setP(SmartDashboard.getNumber(name + "P value", getP()));
        setI(SmartDashboard.getNumber(name + "I value", getI()));
        setD(SmartDashboard.getNumber(name + "D value", getD()));
        setIZone(SmartDashboard.getNumber(name + "I Zone value", getIZone()));
        setFF(SmartDashboard.getNumber(name + "FF value", getFF()));
    }

    /**
     * Gets the relative encoder of the motor
     *
     * @return The relative encoder
     */
    public RelativeEncoder getEncoder(){
        return motorEncoder;
    }

    /**
     * Gets the motor from the DreadbotMotor object
     *
     * @return The motor of the DreadbotMotor object
     */
    public CANSparkMax getSparkMax(){
        return motor;
    }

    /**
     * Gets the PID controller of the motor
     *
     * @return The PID controller
     */
    public SparkMaxPIDController getPIDController(){
        return motorPIDController;
    }

    /**
     * Disables this motor, stopping most of its functions
     */
    public void disable(){
        isDisabled = true;
    }

    /**
     * Checks if this motor is disabled
     *
     * @return The status of the motor, true being disabled
     */
    public boolean isDisabled(){
        return isDisabled;
    }

    /**
     * Prints out the function that caused the motor to be disabled
     *
     * @param errorTrace The function that caused the motor to be disabled
     */
    private void printError(String errorTrace){
        logger.warning(name + "was disabled by " + errorTrace + "()");
    }
}