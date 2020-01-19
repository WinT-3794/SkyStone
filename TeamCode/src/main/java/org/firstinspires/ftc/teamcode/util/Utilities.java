package org.firstinspires.ftc.teamcode.helpers;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Utilities {
    public static final String TABLE_NAME = "times";
    public static final String DATABASE_NAME = "ai";
    public static final int DATABASE_VERSION = 1;
    public static final String ID_INDEX = "id";
    public static final String SCAN_INDEX = "scan";
    public static final String HALF_INDEX = "half";
    public static final String FINAL_INDEX = "final";
    
    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    
    public static final String LABEL_SS = "Stone Target";
    public static final String LABEL_S = "Stone";

    public static final String CREATE_TABLE = "CREATE TABLE "+ TABLE_NAME +" ("+ ID_INDEX +" INTEGER PRIMARY KEY, "+ SCAN_INDEX +" INTEGER, "+ HALF_INDEX +" INTEGER, "+ FINAL_INDEX +" INTEGER)";
    public static final String DROP_TABLE = "DROP TABLE IF EXISTS " + TABLE_NAME;

    public static final double STONE_LENGTH = 8.4;
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = true;
    public static final String VUFORIA_KEY =
            "Aeo+JLn/////AAABmQTMVlw5iEfJhaUjPQ6bUy2N/PXQRu8ZQI/GFuUOEJ/PfS2kHFrx5Jjtd9G9xen+lJZRyrkuFm1MMYmwM0YAz4zYjtw8OYSopoZFXao/1y9IYNCuEjvCeSggPE1bJB8ALdYP+LYWJ7U6AQYxgM2isZ+Il4o6rHNoLLF1lG4r1hCGrsiE4wmLvsxiEf4cQ/UPM2h5jhduEyK6O+lfX1X1TQtqPOOcKNYC+3gYhblPLVt+EYCOclBffeMph1T6bCn8oNKr0lCLMCMs9NBJup6iiTJzq6PNP7+3r17JH/XFRYVtllbGK+FEDTHOqi441KB7Xv2L/6y3tfL+96/JUCu5XsAaeUfipT9W0pKcOuQtU2+A";
    public static final float MM_PER_INCH = 25.4f;
    public static final float STONE_Z = 2.00f * MM_PER_INCH;

    public static final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * Utilities.MM_PER_INCH;
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * Utilities.MM_PER_INCH;
    public static final float CAMERA_LEFT_DISPLACEMENT = 0;

    public static final double RED_COLOR = 0.05;
    public static final double RED_VERIFIER_COLOR = 0.01;

    public static final double BLUE_COLOR = 0.035;
    public static final double BLUE_VERIFIER_COLOR = 0.006;

    public static final double WIDTH = 16.16;
    public static final int CPR = 28;
    public static final int GEAR_RATIO = 20;
    public static final double DIAMETER = 2.952755906;
    public static final double CPI = (CPR * GEAR_RATIO) / (Math.PI * DIAMETER);
    public static final double BIAS = 0.91;
    public static final double MECCY_BIAS = 0.9;
    public static final double CONVERTION = CPI * BIAS;
}
