package it.cnr.isti.wnlab.indoornavigation.android.graphics;

import android.graphics.Canvas;
import android.graphics.Paint;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;

public class XYPositionDrawable implements IndoorDrawable {

    private XYPosition position;
    private Paint paint;
    private float radius;

    public XYPosition getPosition() {
        return position;
    }

    public void setPosition(XYPosition position) {
        this.position = position;
    }

    public XYPositionDrawable(XYPosition position, int color) {
        // Set drawable's position
        this.position = position;
        // Set width and height
        radius = 0.5f;
        // Instantiate paint object
        this.paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        this.paint.setColor(color);
        this.paint.setStyle(Paint.Style.FILL);
    }

    @Override
    public void draw(Canvas canvas) {
        canvas.drawCircle(position.x,position.y,radius,paint);
    }
}
