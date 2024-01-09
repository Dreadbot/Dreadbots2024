package util.misc;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.Timestamp;
import java.util.Date;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class DreadbotPowerLogger {
    private FileWriter fileWriter;
    private PowerDistribution pwrDistro;
    private double startingCurrent = 40.0d;

    public DreadbotPowerLogger(int pdpPort) {
        pwrDistro = new PowerDistribution(pdpPort, ModuleType.kRev);

        try {
            fileWriter = new FileWriter("/C/PowerLog:" + new Date() + new Date().getTime() + ".txt");
                fileWriter.write("--PDP Power log--\n");
                fileWriter.write("Port Number:,");
                for(int i = 0; i < 24; i++){
                    fileWriter.write(i + ",");
                }
                   
                fileWriter.write("\n");
        } catch (Exception e) {
            System.err.println("Creation of log failed, check stack trace below");
            e.printStackTrace();
        }
    }

    public DreadbotPowerLogger(int pdpPort, double startingCurrent){
        this(pdpPort);
        this.startingCurrent = startingCurrent;
    }


    private void logCurrents(){
        if(pwrDistro.getTotalCurrent() >= startingCurrent) {
                String powerOutput = "Power output:,";
            for(int i = 0; i < 24; i++){
                powerOutput += pwrDistro.getCurrent(i) + ",";
            }  

            try {
                fileWriter.write(new Timestamp(new Date().getTime()) + " " + powerOutput + "\n");
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public void stopLogging(){
        try {
            if (fileWriter != null) {
                fileWriter.flush();
                fileWriter.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}