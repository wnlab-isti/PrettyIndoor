package it.cnr.isti.wnlab.indoornavigation.androidapp;

import java.io.IOException;
import java.io.Writer;

import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.observer.Observer;

public class PositionLogger implements Observer<IndoorPosition> {

    protected Writer mWriter;

    public PositionLogger(Writer writer) {
        mWriter = writer;
    }

    @Override
    public void notify(IndoorPosition data) {
        try {
            mWriter.write(data + "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
