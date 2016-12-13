package it.cnr.isti.wnlab.indoornavigator.android;


import android.os.Environment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;

import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.Observer;

public class StepLoggerObserver implements Observer<IndoorPosition> {

    private final String APP_NAME = "PrettyIndoorNavigator";
    public final String POSITION_LOG_FILE_NAME = "pin_positions.log";

    private BufferedWriter mWriter;

    public StepLoggerObserver() {
        // Initialize log writer
        File logFile = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + APP_NAME + "/" + POSITION_LOG_FILE_NAME);
        try {
            mWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(logFile)));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void notify(IndoorPosition data) {
        if(mWriter != null) {
            try {
                mWriter.write(data + "\n");
                mWriter.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
