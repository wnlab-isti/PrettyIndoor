package it.cnr.isti.wnlab.indoornavigation.javaonly.map;

public abstract class RectangleRoomMap extends RoomMap {

    private final float x;
    private final float y;
    private final float width;
    private final float height;

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getWidth() {
        return width;
    }

    public float getHeight() {
        return height;
    }

    public RectangleRoomMap(float x, float y, float width, float height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    @Override
    public boolean isValid(float x, float y) {
        if(x >= this.x && x < this.x+width
            && y >= this.y && y < this.y+height)
            return true;

        return false;
    }

}
