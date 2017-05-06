package it.cnr.isti.wnlab.indoornavigation.log;

import java.io.IOException;
import java.io.Writer;

import it.cnr.isti.wnlab.indoornavigation.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * A DataLogger observes data emission and writes received information on a file.
 * @param <T>
 */
public class DataLogger<T extends RawData> implements DataObserver<T> {

    private Writer mWriter;

    public DataLogger(Writer writer) {
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
