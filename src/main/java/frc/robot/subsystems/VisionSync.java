package frc.robot.subsystems;

import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSync extends SubsystemBase {
    private final DigitalOutput interruptor = new DigitalOutput(9);
    private final Counter syncCounter;
    private NavigableMap<Long, Long> frameIndexTimestamps = new TreeMap<Long, Long>();
    private long lastBuffer;

    public VisionSync() {
        AnalogTrigger trigger = new AnalogTrigger(0);
        trigger.setLimitsVoltage(1.0, 2.5);
        this.syncCounter = new Counter(trigger);
        syncCounter.setPulseLengthMode(1.0);
        lastBuffer = syncCounter.get();
    }

    @Override
    public void periodic() {
        interruptor.pulse(0.001); // pulse to notify for cycle reset
        long newCount = syncCounter.get();
        for (long i = lastBuffer; i < newCount; i++) {
            frameIndexTimestamps.put(i, RobotController.getFPGATime());
        }
        SmartDashboard.putNumber("Current Frame Id", newCount);
        lastBuffer = newCount;
    }

    public long takeTimestampForFrameIndex(long index) {
        Long timestamp = frameIndexTimestamps.remove(index);
        return (timestamp == null) ? RobotController.getFPGATime() : timestamp;
    }

    public long peekTimestampForFrameIndex(long index) {
        return frameIndexTimestamps.getOrDefault(index, RobotController.getFPGATime());
    }
}
