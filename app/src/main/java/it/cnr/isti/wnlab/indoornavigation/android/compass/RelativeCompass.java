package it.cnr.isti.wnlab.indoornavigation.android.compass;

import android.util.Log;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.Acceleration;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.AngularSpeed;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;

/**
 * FOR THE NORTH! (Elaborates an initial direction to refer to as the North.)
 * Given heading is: - <- 0 -> +
 */
public class RelativeCompass extends LawitzkiCompass {

    // Constants
    private final static float ONE_DEGREE_IN_RADIANTS = (float) Math.toRadians(1.f);
    private final static float TOLERANCE = 10.f * ONE_DEGREE_IN_RADIANTS;
    private final static int CALIBRATION_COUNTER_MAX = 80;
    private final static int CALIBRATION_RATE = 60;
    private final static int OPERATIVE_RATE = 500;

    // Variables for calibration
    private boolean calibration = true;
    private int counter;
    private float lastCalibrationHeading;

    public RelativeCompass(
            DataEmitter<Acceleration> accelerometer,
            DataEmitter<AngularSpeed> gyroscope,
            DataEmitter<MagneticField> magnetometer
    ) {
        super(accelerometer, gyroscope, magnetometer, CALIBRATION_RATE);
    }

    @Override
    protected void onHeadingChange(float newHeading, long timestamp) {
        if(calibration) {
            Log.d("COMPASSCAL", "heading: " + counter);
            // Check if newHeading is legal for counting (belongs to [H-1; H+1])
            if(lastCalibrationHeading < (newHeading - TOLERANCE) || lastCalibrationHeading > (newHeading + TOLERANCE)) {
                // Reset counter
                counter = 0;
                // Replace last heading
                lastCalibrationHeading = newHeading;
            } else
                counter++;

            // Calibrate until calibration count max is reached
            if(counter == CALIBRATION_COUNTER_MAX) {
                calibration = false;
                setRate(OPERATIVE_RATE);
            }
        } else {
            // If not calibrating, update heading with correction
            float correctHeading = newHeading-lastCalibrationHeading;
            if(correctHeading < -Math.PI)
                correctHeading += 2.f*Math.PI;
            else if(correctHeading > Math.PI)
                correctHeading -= 2.f*Math.PI;
            // Notify to observers correct heading
            super.onHeadingChange(correctHeading, timestamp);
            //Log.d("COMPASS", "heading: " + correctHeading);
        }
    }
}
