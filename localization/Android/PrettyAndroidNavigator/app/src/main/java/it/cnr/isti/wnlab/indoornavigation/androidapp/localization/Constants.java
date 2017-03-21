package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.os.Environment;

import it.cnr.isti.wnlab.indoornavigation.R;

public final class Constants {

    private Constants() {}

    /*
     * App
     */

    public static String APP_FOLDER_PATH_EXTERNAL_STORAGE = Environment.getExternalStorageDirectory() + "/" + R.string.app_name;

    /*
     * Localization
     */

    public static final float INITIAL_X = 19.5f;
    public static final float INITIAL_Y = 5.7f;
    public static final int INITIAL_FLOOR = 0;

}
