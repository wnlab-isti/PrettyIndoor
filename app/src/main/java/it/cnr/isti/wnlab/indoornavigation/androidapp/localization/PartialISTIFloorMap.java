package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.util.Log;

import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.RoomMap;

public class PartialISTIFloorMap extends FloorMap {

    private static RoomMap westernCorridor = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            if(x >= 0.f && y >= 4.8f // Bottom left
                    && x < 16.8f && y < 6.6f) // Up right
                return true;
            else
                return false;
        }
    };

    private static RoomMap westernDoors = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            if(x >= 16.8f && y >= 4.8f // Bottom left
                    && x < 19.8f && y < 6.6f) // Up right
                return true;
            else
                return false;
        }
    };

    private static RoomMap vendingMachines = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            if(x >= 19.2f && y >= 4.2f // Bottom left
                    && x < 24.6f && y < 8.4f) // Up right
                return true;
            else
                return false;
        }
    };

    private static RoomMap easternHorizontalCorridor = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            if(x >= 24.0f && y >= 4.8f // Bottom left
                    && x < 34.8f && y < 6.6f) // Up right
                return true;
            else
                return false;
        }
    };

    private static RoomMap easternVerticalCorridor = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            if(x >= 34.8f && y >= 4.8f // Bottom left
                    && x < 36.6f && y < 29.4f) // Up right
                return true;
            else
                return false;
        }
    };

    public PartialISTIFloorMap() {
        super(
                westernCorridor,
                westernDoors,
                vendingMachines,
                easternHorizontalCorridor,
                easternVerticalCorridor
        );
    }

    @Override
    public boolean isValid(float x, float y) {
        if(super.isValid(x,y)) {
            Log.d("PFS", "Valid position");
            return true;
        } else {
            Log.d("PFS", "Invalid position");
            return false;
        }
    }
}
