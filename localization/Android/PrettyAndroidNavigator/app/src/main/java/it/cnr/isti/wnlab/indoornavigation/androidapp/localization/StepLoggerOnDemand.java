package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;


import android.util.Log;

import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;

public class StepLoggerOnDemand implements Closeable {

    public final String POSITION_LOG_FILE_NAME = "positions" + System.currentTimeMillis() + ".log";
    private BufferedWriter mWriter;
    private IndoorLocalizationStrategy mStrategy;

    public StepLoggerOnDemand(IndoorLocalizationStrategy strategy, String logFolderPath) throws IOException {
        // Initialize log file and writer
        File logFile = new File(logFolderPath + "/" + POSITION_LOG_FILE_NAME);
        mWriter = new BufferedWriter(new FileWriter(logFile));

        // Set current localization strategy
        mStrategy = strategy;
    }

    public void log() {
        IndoorPosition position = mStrategy.getCurrentPosition();
        Log.d("IPF", "Logging on file: " + position);
        try {
            mWriter.write(position + "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void close() throws IOException {
        mWriter.close();
    }
}
