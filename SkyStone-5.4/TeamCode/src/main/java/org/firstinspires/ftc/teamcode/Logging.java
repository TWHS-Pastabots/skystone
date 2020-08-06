package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

public class Logging implements Runnable {

    //Thread run condition
    private boolean isRunning = true;
    private int sleepTime = 250;
    private HashMap<String, String> logs = new HashMap<>();
    private ArrayList<File> files = new ArrayList<>();

    public void stop(){
        isRunning = false;
    }

    public void createLog(String name){
        logs.put(name + ".txt", "");
        files.add(AppUtil.getInstance().getSettingsFile(name + ".txt"));
    }

    public void add(String name, String content){
        logs.put(name + ".txt", logs.get(name + ".txt") + content );
    }

    private void writeLogs(){
        for(File file : files){
            ReadWriteFile.writeFile(file, logs.get(file.getName()));
        }
    }

    @Override
    public void run() {
        while(isRunning){
            writeLogs();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        writeLogs();
    }


}
