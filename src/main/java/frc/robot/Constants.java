/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Flag that tells the code systems exist
     */
    public static final boolean DRIVE_AVAILABLE         = true;
    public static final boolean CAMERA_AVAILABLE        = false;
    public static final boolean AIM_AVAILABLE           = false;
    public static final boolean TURRET_AVAILABLE        = true;
    public static final boolean MAGAZINE_AVAILABLE      = true;
    public static final boolean PICKUP_AVAILABLE        = true;
    public static final boolean KICKER_AVAILABLE        = false;
    public static final boolean CLIMBER_AVAILABLE       = false;
    public static final boolean CONTROLPANEL_AVAILABLE  = false;
    public static final boolean COMPRESSOR_AVAILABLE    = true;
    public static final boolean GYRO_AVAILABLE          = false;
    public static final boolean BUTTONBOX_AVAILABLE     = false;

    /** Enum to hold all information about pneumatic solenoids */
    public enum PneumaticChannel {
        DRIVE_LOW_GEAR(0),
        DRIVE_HIGH_GEAR(1),
        PICKUP_RETRACT(2),
        PICKUP_EXTEND(3),
        //OPEN_ACCELERATOR_FLAPS(4),
        //CLOSE_ACCELERATOR_FLAPS(5);
        CLIMBER_RELEASE(4);
        private final int channel;
        private PneumaticChannel(final int ch) {
            channel = ch;
        }
        public int getChannel() {
            return channel;
        }
    }

    /**
     * Enum to hold all information about devices on the CAN bus
     */
    public enum CANIDs {
        DRIVE_LEFT_LEADER    (20, true, true), 
        DRIVE_LEFT_FOLLOWER  (21, true, false),
        DRIVE_RIGHT_LEADER   (10, false, true), 
        DRIVE_RIGHT_FOLLOWER (11, false, false), 

        TURRET_DIRECTION     (30, false, false),
        TURRET_SHOOTER       (28, true, false),

        PICKUP               (60, true, false),
        MAGAZINE             (41, false, false),

        CLIMBER_RIGHT        (50, false, false),
        CLIMBER_LEFT         (51, false, false);

        private final int canid;
        private final boolean inverted;
        private final boolean leader;

        CANIDs(final int id, final boolean inverted, final boolean leader) {
            this.canid = id;
            this.inverted = inverted;
            this.leader = leader;
        }

        public int getid() {
            return canid;
        }

        public boolean isFollower() {
            return !leader;
        }

        public boolean isInverted() {
            return inverted;
        }
    }

    /**
     * Used to set the mode of the turret
     */
    public enum AutoMode {
        AUTO, 
        MANUAL;
    }

    /**
     * Switches the gears of the drive
     */
    public enum Gear {
        LOW, 
        HIGH;
    }

    /**
     * Used in Aim to setNoTargetMode
     */
    public enum Mode {
        DONT_MOVE, 
        SCAN, 
        FACE_FORWARD;
    }

    public enum SmartPID {
        TURRET_DIRECTION(0.005, 0.9e-5, 0.0, 0.0, 0.0, -2.0, 2.0, 0.0, 0.0, 0.0, 0.0),
        TURRET_HOOD(0.005, 1.1e-5, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0);

        private double p, i, d, iz, ff, minOut, maxOut, maxVel, minVel, maxAcc, allowedErr;

        private SmartPID(double newP, double newI, double newD, double newIz, double newFF, double newMinOut, double newMaxOut, double newMaxVel, double newMinVel, double newMaxAcc, double newAllowedErr) {
            p = newP;
            i = newI;
            d = newD;
            iz = newIz;
            ff = newFF;
            minOut = newMinOut;
            maxOut = newMaxOut;
            maxVel = newMaxVel;
            minVel = newMinVel;
            maxAcc = newMaxAcc;
            allowedErr = newAllowedErr;
        }

        public double getP() {
            return p;
        }
        
        public double getI() {
            return i;
        }

        public double getD() {
            return d;
        }
        
        public double getIz() {
            return iz;
        }

        public double getFF() {
            return ff;
        }

        public double getMinOut() {
            return minOut;
        }

        public double getMaxOut() {
            return maxOut;
        }

        public double getMaxVel() {
            return maxVel;
        }

        public double getMinVel() {
            return minVel;
        }

        public double getMaxAcc() {
            return maxAcc;
        }

        public double getAllowedErr() {
            return allowedErr;
        }
    }

    public enum PID {
        //TURRET_DIRECTION(0, 0, 0),
        TURRET_SHOOTER_SPEED(5e-5, 1e-6, 5e-5),
        TURRET_HOOD(0.005, 1.1e-5, 0.0),
        DRIVE_LEFT(0.259, 0, 0),
        DRIVE_RIGHT(0.259, 0, 0);

        private double p, i, d;

        private PID(double newP, double newI, double newD) {
            p = newP;
            i = newI;
            d = newD;
        }

        public double getP() {
            return p;
        }
        
        public double getI() {
            return i;
        }

        public double getD() {
            return d;
        }
    }
    /*----------------------------------------------------------------------------*/ 
    /*  Pixy Camera on front of used to find Power Cells                          */
    /*----------------------------------------------------------------------------*/ 
    public static final int PIXY_ANALOG_CHANNEL = 0;

    //Drive
    public static final int USB_STICK_PILOT = 0;
    public static final int USB_STICK_COPILOT1 = 1;
    public static final int USB_STICK_COPILOT2 = 2;

    public static final double DRIVE_DEAD_BAND = 0.1;

    //Speed/Voltage for subsystems
    public static final double PICKUP_ROLLER_SPEED = 0.4;
    public static final double KICKER_SPEED = 0.5;
    public static final double MAGAZINE_LOAD_SPEED = 0.3;
    public static final double MAGAZINE_SHOOT_SPEED = 0.3;

     //used for climber currently not being used
     public static final int SOLENOID_EXTEND = 1;
     public static final int SOLENOID_RETRACT = 2;
     

     // public static final double PICKUP_TIME = 0.5;
     public static final double EXTEND_TIME = 0.5;
     public static final double RETRACT_TIME = 0.5;
     public static final double SHOOTER_TIME = 0.7;

     //Speeds for Turret Direction
     public static final double TURRET_MANUAL_SPEED_FAST = 10.0;//// was 3.5;
     public static final double TURRET_MANUAL_SPEED_SLOW = 1.5;

     public static final double TURRET_DIRECTION_SETPOINT = 160.0;

     // direction encoder = 42 counts per rev, 64:1 gearbox ratio, 92 teeth on turret, 18 teeth on motor
     // 92 / 18 * 64 * 42 = 13739 counts per turret rev, / 360 deg = 38.2 counts / deg
     // camera in X = 320 pixels, 75 deg
     // 320 / 75 = 4.27 pixels / deg
     // 38.2 (counts / deg) / 4.27 (pixels / deg) = 8.9 counts / pixel (direction)
     public static final double TURRET_DIRECTION_COUNTS_PER_PIXEL = 8.9;

     public static final float TURRET_DIRECTION_FWD_LIMIT = 125.0f;
     public static final float TURRET_DIRECTION_REV_LIMIT = -125.0f;
     public static final double TURRET_DIRECTION_POS_CONVERSION_FACTOR = 1.0;

     //Hood positions
     public static final double TURRET_HOOD_CLOSE = 200.0;
     public static final double TURRET_HOOD_MEDIUM = 100.0;
     public static final double TURRET_HOOD_FAR = 50.0;
     public static final double TURRET_HOOD_VOLTAGE = 0.4;

     public static final double TURRET_HOOD_MIDPOINT = 100.0;

     // hood encoder = 42 counts per rev, 64:1 (subject to change) gearbox ratio, teeth ratio is 1-to-1
     // (42 / 42 = 1) * 64 * 42 = 2688 counts per timing belt rev (hood) / 360 deg = 7.47 counts / deg
     // camera in Y = 200 pixels, 47 deg
     // 200 / 47 = 4.26 pixels / deg
     // 7.47 (counts / deg) / 4.26 (pixels / deg) = 1.8 counts / pixel
     public static final double TURRET_HOOD_COUNTS_PER_PIXEL = 1.8;

     public static final double TURRET_HOOD_INIT_POWER = 1.0;
     public static final double TURRET_HOOD_CURRENT_LIMIT = 1.0;

     public static final float TURRET_HOOD_FWD_LIMIT = 1000.0f;
     public static final float TURRET_HOOD_REV_LIMIT = -1000.0f;

     //Speed for Shooter
     
     public static final double SHOOTER_SPEED_LOWER_HUB  =  1950.0;    
     public static final double SHOOTER_SPEED_TARMAC     =  2650.0;
     public static final double SHOOTER_SPEED_CARGO_RING =  3200.0;
     public static final double SHOOTER_SPEED_PAD1       =  3900.0;
     
     
 
    //used in Turret sets port for gyro
    public static final SPI.Port CHASSIS_GYRO_PORT = SPI.Port.kOnboardCS0;

    //subsystem PIDs

    public static final double kP_Shooter                   =  5e-4;     //   5e-4
    public static final double kI_Shooter                   =  1e-6;     //
    public static final double kD_Shooter                   =  0.0;      //
    public static final double klz_Shooter                  =  0.0;      //
    public static final double kFF_Shooter                  =  0.0;      //
    public static final double kMaxOutput_Shooter           =  1.0;      //
    public static final double kMinOutput_Shooter           = -1.0;     //

    public static final double kP_Turret_Camera             =  0.024;    // was 0.005
    public static final double kI_Turret_Camera             =  0.0008;   // was 0.0008
    public static final double kD_Turret_Camera             =  0.0;      //

    public static final double kP_Turret_Position           =  0.015;    // was 0.005
    public static final double kI_Turret_Position           =  0.0;   // was 0.0008
    public static final double kD_Turret_Position           =  0.0;      //
    public static final double klz_Turret_Position          =  0.0;      //
    public static final double kFF_Turret_Position          =  0.0;      //
    public static final double kMaxOutput_Turret_Position   =  1.0;      //
    public static final double kMinOutput_Turret_Position   = -1.0;      //

    //Drive PIDs
    public static final double kP_Drive_Gyro                =  0.008;    //
    public static final double kI_Drive_Gyro                =  0.0;      //
    public static final double kD_Drive_Gyro                =  0.0;      //

    public static final double kP_Drive_Pixy                =  0.08;      //  was .1
    public static final double kI_Drive_Pixy                =  0.0;      //
    public static final double kD_Drive_Pixy                =  0.0;      //

    /*----------------------------------------------------------------------------*/ 
    /*                                                                            */
    /*----------------------------------------------------------------------------*/ 
    public static final int PILOT               =  0;
    public static final int PILOT_BUTTON_1      =  1;
    public static final int PILOT_BUTTON_2      =  2;
    public static final int PILOT_JOYSTICK_FOLLOW_POWER_CELL_BUTTON    =  2;
    public static final int PILOT_BUTTON_3      =  3;
    public static final int PILOT_BUTTON_4      =  4;
    public static final int PILOT_BUTTON_5      =  5;
    public static final int Pilot_Button_6      =  6;
    public static final int PILOT_BUTTON_7      =  7;
    public static final int PILOT_BUTTON_8      =  8;
    public static final int PILOT_CLIMB_DEPLOYx  =  9;
    public static final int PILOT_CLIMBx         = 10;
    public static final int PILOT_BUTTON_11     = 11;

    public static final int COPILOT1  =  1;
    public static final int COPILOT1_JOYSTICK_FOLLOW_POWER_CELL_BUTTON =  1;
    public static final int COPILOT1_PICKUP                            =  2;
    public static final int COPILOT1_MAGAZINE_DOWN                     =  3;
    public static final int COPILOT1_SHOOT_TARMAC                      =  4;
    public static final int COPILOT1_TURRET_LEFT                       =  5;
    public static final int COPILOT1_TURRET_FINE_ADJUST                =  6;
    public static final int COPILOT1_TURRET_RIGHT                      =  7;
    public static final int COPILOT1_SHOOT                             =  8;
    public static final int COPILOT1_SHOOT_STOP                        =  9;
    public static final int COPILOT1_SHOOT_PAD1                        = 10;

    public static final int COPILOT2             =  2;
    public static final int COPILOT2_SHOOT_CARGO =  1;      // Button 11
    public static final int COPILOT2_MAGAZINE_UP =  2;      // Button 12
    public static final int COPILOT2_CLIMBER_RETRACT     =  3;      // Button 13
    public static final int COPILOT2_ClIMBER_DEPLOY      =  4;      // Button 14
    public static final int COPILOT2_PICKUP_REV  =  5;     
    public static final int COPILOT2_LOWER_HUB   =  6;
    public static final int COPILOT2_ACELL_REV   =  7;
    public static final int COPILOT2_ACCEL_FWD   =  8;
    public static final int COPILOT2_BUTTON_9    =  9;
    public static final int COPILOT2_BUTTON_10   = 10;

    public static final int COPILOT1_PICKUP_ROLLERS = 11; //FIXME: Needs a physical button and button ID, this was made purely to remove errors

}
