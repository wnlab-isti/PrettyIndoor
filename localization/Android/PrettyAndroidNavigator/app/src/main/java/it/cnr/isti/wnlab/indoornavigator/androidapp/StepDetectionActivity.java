package it.cnr.isti.wnlab.indoornavigator.androidapp;

import android.content.Context;
import android.graphics.Color;
import android.graphics.Paint;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import android.widget.SeekBar;
import android.widget.TextView;

import it.cnr.isti.wnlab.indoornavigator.R;
import it.cnr.isti.wnlab.indoornavigator.android.handlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigator.android.stepdetection.FasterStepDetector;
import it.cnr.isti.wnlab.indoornavigator.observers.Observer;
import it.cnr.isti.wnlab.indoornavigator.types.Step;

public class StepDetectionActivity extends AppCompatActivity implements SeekBar.OnSeekBarChangeListener {

    private int sensitivity;
    private float MAX_SENSITIVITY = 5.5f;
    private float MIN_SENSITIVITY = 1.35f;

    @Override
    public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
        sensitivity = i;
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {
        float newsens = ((MAX_SENSITIVITY-MIN_SENSITIVITY)*((sensitivity)/100.f))+MIN_SENSITIVITY;
        sd.setSensitivity(newsens);
        tv.setText(newsens+"");
    }

    /**
     * Test map
     */
    private class TestView extends View {

        Paint paint = new Paint();
        public float x,y;
        public float offsetX, offsetY;

        // Colors
        private int[] colors = { Color.RED, Color.GREEN, Color.BLUE };
        private int counter = -1;

        public TestView(Context context) {
            super(context);
            paint.setAntiAlias(true);
            paint.setStyle(Paint.Style.FILL);
            paint.setColor(colors[0]);
        }

        private void updateColor() {
            counter = (counter+1)%colors.length;
            setBackgroundColor(colors[counter]);
            Log.d("SDTEST","New color is " + counter);
            invalidate();
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

    private FasterStepDetector sd;
    private TestView mView;
    private TextView tv;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_step_detection);

        // Initialize view
        mView = new TestView(getApplicationContext());
        ViewGroup insertPoint = (ViewGroup) findViewById(R.id.activity_step_detection);
        insertPoint.addView(mView, 0, new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 50));
        mView.updateColor();

        // Initialize step detector
        AccelerometerHandler ah = new AccelerometerHandler((SensorManager) getSystemService(SENSOR_SERVICE), SensorManager.SENSOR_DELAY_FASTEST);
        sd = new FasterStepDetector(ah);
        sd.register(new Observer<Step>() {
            @Override
            public void notify(Step data) {
                mView.updateColor();
            }
        });
        ah.start();

        // Initialize Seekbar
        ((SeekBar) findViewById(R.id.seekBar)).setOnSeekBarChangeListener(this);

        // Initialize textview
        tv = (TextView) findViewById(R.id.textView);
    }

}
