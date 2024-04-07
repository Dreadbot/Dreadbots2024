package util.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;

public class GyroIONavX implements GyroIO {
    private AHRS gyro;

    public GyroIONavX() {
        gyro = new AHRS(Port.kMXP);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = gyro.getRotation2d();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
