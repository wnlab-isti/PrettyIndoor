package it.cnr.isti.wnlab.indoornavigator.android;

import java.io.IOException;
import java.io.Writer;

import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.PositionUpdateCallback;

public class PositionLogCallback implements PositionUpdateCallback {

    private Writer mWriter;

    public PositionLogCallback(Writer writer) {
        mWriter = writer;
    }

    @Override
    public void onPositionUpdate(IndoorPosition newPosition) {
        try {
            Logger.steplog(mWriter, newPosition);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
