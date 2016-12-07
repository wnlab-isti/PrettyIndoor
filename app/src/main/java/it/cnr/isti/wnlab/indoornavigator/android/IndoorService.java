package it.cnr.isti.wnlab.indoornavigator.android;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.Bundle;
import android.os.IBinder;

import java.util.List;

import it.cnr.isti.wnlab.indoornavigator.androidutils.IndoorNavigator;
import it.cnr.isti.wnlab.indoornavigator.framework.PositionUpdateCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.StartableStoppable;

public class IndoorService extends Service {

    // IndoorNavigator's creation

    public static final String BUNDLE_HANDLERS = "INDOORHANDLERS";
    public static final String BUNDLE_UPDATER = "INDOORUPDATER";

    private IndoorNavigator mNavigator;

    private IndoorNavigator initNavigator(Bundle bundle) {
        // Build navigator
        IndoorNavigator.Builder builder = new IndoorNavigator.Builder();
        builder.setSources((List<StartableStoppable>) bundle.get(BUNDLE_HANDLERS));
        builder.setPositionUpdater((PositionUpdateCallback) bundle.get(BUNDLE_UPDATER));
        return builder.create();
    }

    // Service's stuff

    private final Binder mBinder = new IndoorServiceBinder();

    public class IndoorServiceBinder extends Binder {
        public IndoorNavigator getNav() {
            return mNavigator;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        if(mNavigator == null)
            mNavigator = initNavigator(intent.getExtras());
        return mBinder;
    }
}
