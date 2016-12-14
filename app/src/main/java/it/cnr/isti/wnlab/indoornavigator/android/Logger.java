package it.cnr.isti.wnlab.indoornavigator.android;

import java.io.IOException;
import java.io.Writer;

import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.types.RawData;

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
