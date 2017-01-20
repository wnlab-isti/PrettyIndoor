package it.cnr.isti.wnlab.indoornavigation.androidapp;

import java.io.IOException;
import java.io.Writer;

import it.cnr.isti.wnlab.indoornavigation.observers.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

public class Logger<T extends RawData> implements DataObserver<T> {

    private Writer mWriter;

    public Logger(Writer writer) {
        mWriter = writer;
    }

    @Override
    public void notify(T data) {
        try {
            mWriter.write(data + "\n");
        } catch(IOException e) {
            e.printStackTrace();
        }
    }
}
