package edu.ahs.robotics.hardware;

public class ChassisMotors {

    public static enum Mecanum {
        FRONTLEFT("FL"), FRONTRIGHT("FR"), BACKLEFT("BL"), BACKRIGHT("BR");

        private String deviceName;

        Mecanum(String deviceName) {
            this.deviceName = deviceName;
        }

        public String getDeviceName() {
            return deviceName;
        }
    }
    public static enum PushBot{
        LEFT, RIGHT
    }
    public static enum WestCoast{
        FRONTLEFT, FRONTRIGHT, BACKLEFT, BACKRIGHT, MIDLEFT, MIDRIGHT
    }

}
