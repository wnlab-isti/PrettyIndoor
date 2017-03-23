package it.cnr.isti.wnlab.indoornavigation.androidapp.compass;

import android.hardware.SensorManager;

import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigation.android.handlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigation.javaonly.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.Acceleration;

public class YetAnotherCompass extends AbstractEmitter<Heading> implements StartableStoppable {

    private Timer mTimer;
    private MagneticField lastMF;
    private Acceleration lastAcc;

    public YetAnotherCompass(DataEmitter<Acceleration> ah, DataEmitter<MagneticField> mh) {
        ah.register(new DataObserver<Acceleration>() {
            @Override
            public void notify(Acceleration data) {
                lastAcc = data;
                notifyHeading();
            }
        });

        mh.register(new DataObserver<MagneticField>() {
            @Override
            public void notify(MagneticField data) {
                lastMF = data;
                notifyHeading();
            }
        });
    }

    private void notifyHeading() {
        if (lastAcc != null && lastMF != null) {
            float R[] = new float[9];
            float I[] = new float[9];
            float[] gravity = new float[3];
            gravity[0] = lastAcc.x;
            gravity[1] = lastAcc.y;
            gravity[2] = lastAcc.z;
            float[] magnetic = new float[3];
            magnetic[0] = lastMF.x;
            magnetic[1] = lastMF.y;
            magnetic[2] = lastMF.z;
            boolean success = SensorManager.getRotationMatrix(R, I, gravity, magnetic);
            if (success) {
                float orientation[] = new float[3];
                SensorManager.getOrientation(R, orientation);
                notifyObservers(new Heading((float) Math.toDegrees(orientation[0]),System.currentTimeMillis())); // orientation contains: azimut, pitch and roll
            }
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isStarted() {
        return true;
    }
}
