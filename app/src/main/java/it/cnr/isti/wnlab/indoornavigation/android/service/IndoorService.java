package it.cnr.isti.wnlab.indoornavigation.android.service;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.Bundle;
import android.os.IBinder;

import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorNavigator;

public class IndoorService extends Service {

    // AndroidIndoorNavigator's creation

    public static final String BUNDLE_HANDLERS = "INDOORHANDLERS";
    public static final String BUNDLE_UPDATER = "INDOORUPDATER";

    private IndoorNavigator mNavigator;

    private IndoorNavigator initNavigator(Bundle bundle) {
        // TODO
        throw new UnsupportedOperationException("Not implemented yet");
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
