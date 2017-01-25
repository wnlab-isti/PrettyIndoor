package it.cnr.isti.wnlab.indoornavigation.androidapp;


import android.os.Environment;

import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;

import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.observers.Observer;

public class StepLoggerObserver implements Observer<IndoorPosition>, Closeable {

    private final String APP_NAME = "PrettyIndoorNavigator";
    public final String POSITION_LOG_FILE_NAME = "pin_positions" + System.currentTimeMillis() + ".log";
    private BufferedWriter mWriter;
    private IndoorPosition data;

    public StepLoggerObserver(IndoorPosition initialPosition) {
        // Initialize log writer
        File logFile = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + APP_NAME + "/" + POSITION_LOG_FILE_NAME);
        try {
            mWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(logFile)));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        // Set initial position (for avoiding null data)
        data = initialPosition;
    }

    public void steplog() {
        if(mWriter != null) {
            try {
                mWriter.write(data + "\n");
                mWriter.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void notify(IndoorPosition data) {
        this.data = data;
    }

    @Override
    public void close() throws IOException {
        mWriter.close();
    }
}