package it.cnr.isti.wnlab.indoornavigation.map;

import java.util.Arrays;
import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;

public abstract class FloorMap implements XYMap {

    private int floor;
    private Collection<RoomMap> rooms;

    public FloorMap(int floor, RoomMap... rooms) {
        this.floor = floor;
        this.rooms = Arrays.asList(rooms);
    }

    public int getFloor() {
        return floor;
    }

    @Override
    public boolean isValid(float x, float y) {
        for(RoomMap room : rooms) {
            if(room.isValid(x,y))
                return true;
        }
        return false;
    }

    public abstract XYPosition nearestValid(float x, float y);
}
