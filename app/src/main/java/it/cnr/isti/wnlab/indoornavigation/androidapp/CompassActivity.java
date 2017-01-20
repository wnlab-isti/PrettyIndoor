package it.cnr.isti.wnlab.indoornavigation.androidapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.hardware.SensorManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.android.compass.Compass;
import it.cnr.isti.wnlab.indoornavigation.android.compass.LawitzkiCompass;
import it.cnr.isti.wnlab.indoornavigation.android.compass.RelativeCompass;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.MagneticFieldHandler;
import it.cnr.isti.wnlab.indoornavigation.observers.Observer;
import it.cnr.isti.wnlab.indoornavigation.types.Heading;

public class CompassActivity extends AppCompatActivity {

    /**
     * Test map
     */
    private class TestView extends View {

        Paint paint = new Paint();
        public float x,y;
        public float rotate;
        private float w = 100.f;
        private float h = 300.f;
        public float offsetX, offsetY;
        private float factor = 90.f;

        public TestView(Context context) {
            super(context);
            paint.setAntiAlias(true);
            paint.setColor(Color.RED);
            paint.setStyle(Paint.Style.FILL);
        }

        @Override
        public void onDraw(Canvas c) {
            c.save();

            c.translate(offsetX+factor*x, offsetY+factor*y);
            c.rotate((float) Math.toDegrees(rotate));
            //c.drawRect(-w/2.f, -h/2.f, w/2.f, h/2.f, paint);
            Path tri = new Path();
            tri.setFillType(Path.FillType.EVEN_ODD);
            tri.moveTo(0.f,-h/2.f);
            tri.lineTo(w/2.f, h/2.f);
            tri.lineTo(-w/2.f,h/2.f);
            tri.lineTo(0.f,-h/2.f);
            tri.close();
            c.drawPath(tri,paint);

            c.restore();
        }

        @Override
        protected void onLayout(boolean changed, int left, int top, int right, int bottom) {
            super.onLayout(changed, left, top, right, bottom);
            if(changed) {
                offsetX = (right-left)/2;
                offsetY = (bottom-top)/2;
            }
        }
    }

    private TestView mView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_compass);

        // Initialize view
        mView = new TestView(getApplicationContext());
        mView.setBackgroundColor(Color.rgb(0,0,0x99));
        ViewGroup insertPoint = (ViewGroup) findViewById(R.id.activity_compass);
        insertPoint.addView(mView, 0, new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));

        // Sensors
        SensorManager manager = (SensorManager) getSystemService(SENSOR_SERVICE);
        int delay = SensorManager.SENSOR_DELAY_FASTEST;
        AccelerometerHandler ah = new AccelerometerHandler(manager, delay);
        GyroscopeHandler gh = new GyroscopeHandler(manager, delay);
        MagneticFieldHandler mh = new MagneticFieldHandler(manager, delay);

        // Gyroscope-only compass
        /*Compass compass = new SimpleGyroCompass(
                new HeadingChangeCallback() {
                    @Override
                    public void onHeadingChange(float newHeading, long timestamp) {
                        updateHeading(newHeading);
                    }
                }, gh);*/

        // Lawitzki-only compass
        /*Compass compass = new LawitzkiCompass(ah, gh, mh, LawitzkiCompass.RATE);
        compass.register(new Observer<Heading>() {
            @Override
            public void notify(Heading newHeading) {
                updateHeading(newHeading.heading);
            }
        });*/
        // Lawitzi relative compass
        Compass compass = new RelativeCompass(ah, gh, mh, LawitzkiCompass.DEFAULT_RATE);
        compass.register(new Observer<Heading>() {
            @Override
            public void notify(Heading newHeading) {
                updateHeading(newHeading);
            }
        });

        // Start everything
        ah.start();
        gh.start();
        mh.start();
    }

    private void updateHeading(Heading newHeading) {
        float heading = newHeading.heading;
        mView.rotate = heading;
        mView.invalidate();
        Log.d("COMPASSACT", "New heading: " + heading);
    }
}
