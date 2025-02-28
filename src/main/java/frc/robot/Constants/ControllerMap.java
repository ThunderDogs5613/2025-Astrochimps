package frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class ControllerMap {

    public static CommandGenericHID driveStick = new CommandGenericHID(0);

    public static CommandGenericHID getDriveStick(){
        if (driveStick == null){
            driveStick = new CommandGenericHID(0);
        }
        return driveStick;
    }

    public static CommandGenericHID throttle = new CommandGenericHID(1);

    public static CommandGenericHID getThrottlCommandGenericHID(){
        if (driveStick == null){
            driveStick = new CommandGenericHID(1);
        }
        return throttle;
    }
    

    public static class DriveController{
        public static class Axis{
            public static final int STICK_X = 0;
            public static final int STICK_Y = 1;
            public static final int STICK_THROT = 2;
            public static final int STICK_ROT = 3;
        }

        public static class Button{
            public static final int TRIGGER = 1;
            public static final int B2 = 2;
            public static final int B3 = 3;
            public static final int B4 = 4;
            public static final int B5 = 5;
            public static final int B6 = 6;
            public static final int B7 = 7;
            public static final int B8 = 8;
            public static final int B9 = 9;
            public static final int B10 = 10;
        }
    }   
}