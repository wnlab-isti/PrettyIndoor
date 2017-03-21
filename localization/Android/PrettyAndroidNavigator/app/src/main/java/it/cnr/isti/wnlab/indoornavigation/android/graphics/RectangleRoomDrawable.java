package it.cnr.isti.wnlab.indoornavigation.android.graphics;

import android.graphics.Canvas;
import android.graphics.Paint;

import it.cnr.isti.wnlab.indoornavigation.javaonly.map.RectangleRoomMap;

public class RectangleRoomDrawable implements IndoorDrawable {

    private RectangleRoomMap room;
    private Paint paint;

    public RectangleRoomDrawable(RectangleRoomMap room, int color) {
        // The room this object wraps
        this.room = room;
        // Paint
        this.paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        this.paint.setColor(color);
        this.paint.setStyle(Paint.Style.FILL);
    }

    @Override
    public void draw(Canvas canvas) {
        float left = room.getX();
        float top = room.getX();
        float right = left + room.getWidth();
        float bottom = right - room.getHeight(); // Minus!!!
        canvas.drawRect(left,top,right,bottom,paint);
    }

}
