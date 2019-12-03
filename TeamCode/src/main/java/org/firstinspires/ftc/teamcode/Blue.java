
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous
public class Blue extends LinearOpMode {
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true ;
    private static final String VUFORIA_KEY =
            "Aeo+JLn/////AAABmQTMVlw5iEfJhaUjPQ6bUy2N/PXQRu8ZQI/GFuUOEJ/PfS2kHFrx5Jjtd9G9xen+lJZRyrkuFm1MMYmwM0YAz4zYjtw8OYSopoZFXao/1y9IYNCuEjvCeSggPE1bJB8ALdYP+LYWJ7U6AQYxgM2isZ+Il4o6rHNoLLF1lG4r1hCGrsiE4wmLvsxiEf4cQ/UPM2h5jhduEyK6O+lfX1X1TQtqPOOcKNYC+3gYhblPLVt+EYCOclBffeMph1T6bCn8oNKr0lCLMCMs9NBJup6iiTJzq6PNP7+3r17JH/XFRYVtllbGK+FEDTHOqi441KB7Xv2L/6y3tfL+96/JUCu5XsAaeUfipT9W0pKcOuQtU2+A";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    Double width = 16.16; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 2.952755906;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.91;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private AndroidTextToSpeech androidTextToSpeech;
    
    private TouchSensor digitalTouch1;
    private TouchSensor digitalTouch2;
    private TouchSensor foundation;
    
    
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;
    private static final float bridgeRotZ = 180;

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private boolean skystoneTarget = false;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private LibTMOA mecanum;

    private DcMotor SP_DRC;
    private DcMotor IN_DRC;
    private DcMotor SP_IZQ;
    private Servo JL_DRC;
    private Servo JL_IZQ;
    private DcMotor IN_IZQ;
    private DcMotor ELE_P;
    private DcMotor BRZ;
    private DcMotor ELE_A1;
    private DcMotor ELE_A2;
    private Servo CUBO;
    private Servo RLQ;
    private NormalizedColorSensor colorSensor;
    
    private double distance = 0;
    private boolean initMoving = true;
    private NormalizedRGBA colors;
    private float blue = 0;
    private int change = 0;
    
    @Override public void runOpMode() {

        initGyro();

        SP_DRC = hardwareMap.dcMotor.get("SP_DRC");
        IN_DRC = hardwareMap.dcMotor.get("IN_DRC");
        SP_IZQ = hardwareMap.dcMotor.get("SP_IZQ");
        JL_DRC = hardwareMap.servo.get("JL_DRC");
        JL_IZQ = hardwareMap.servo.get("JL_IZQ");
        IN_IZQ = hardwareMap.dcMotor.get("IN_IZQ");
        BRZ = hardwareMap.dcMotor.get("BRZ");
        ELE_P = hardwareMap.dcMotor.get("ELE_P");
        ELE_A1 = hardwareMap.dcMotor.get("ELE_A1");
        ELE_A2 = hardwareMap.dcMotor.get("ELE_A2");
        CUBO = hardwareMap.servo.get("CUBO");
        RLQ = hardwareMap.servo.get("RLQ");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        SP_IZQ.setDirection(DcMotorSimple.Direction.REVERSE);
        IN_IZQ.setDirection(DcMotorSimple.Direction.REVERSE);
        ELE_A1.setDirection(DcMotorSimple.Direction.REVERSE);
        ELE_A2.setDirection(DcMotorSimple.Direction.REVERSE);
        BRZ.setDirection(DcMotorSimple.Direction.REVERSE);


        SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ELE_P.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SP_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ELE_P.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        JL_DRC.setPosition(0);
        JL_IZQ.setPosition(1);
        RLQ.setPosition(0);

        mecanum = new LibTMOA(SP_IZQ, IN_IZQ, SP_DRC,IN_DRC);
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        
        colors = colorSensor.getNormalizedColors();
        
        digitalTouch1 = hardwareMap.touchSensor.get("digital_touch1");
        digitalTouch2 = hardwareMap.touchSensor.get("digital_touch2");
        foundation = hardwareMap.touchSensor.get("foundation");
        
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT     = 0;
        

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();
        androidTextToSpeech = new AndroidTextToSpeech();
        androidTextToSpeech.initialize();
        androidTextToSpeech.setLanguageAndCountry("en", "US");
        telemetry.addLine(">> Para continuar");
        androidTextToSpeech.speak("Vuforia initialized.");
        androidTextToSpeech.speak("Ready for begin.");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
            CameraDevice.getInstance().setFlashTorchMode(true);
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    if(trackable.getName().equals("Stone Target")){
                        skystoneTarget = true;
                    } else {
                        skystoneTarget = false;
                    }
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            if(initMoving){
                moveToPosition(23,1);
            }
            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                if(skystoneTarget){
                        //Coninue Scripting
                    if(change == 0){
                        while(Math.abs(rotation.thirdAngle) > 14){
                            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                            mecanum.rotate(0.2,rotation.thirdAngle * Math.PI / 180);
                            if(Math.abs(rotation.thirdAngle) < 14){
                                mecanum.stop();
                                break;
                            }
                        }
                        
                        //distance = Math.abs(DistanceSensor.getDistance(DistanceUnit.MM));
                        telemetry.addData("Paso","Calibración");
                        telemetry.addData("Distancia",distance);
                        telemetry.update();
                        /*while(distance > 340 || distance < 325){
                            if(distance < 340){
                                mecanum.move(.1, Math.PI,0);
                            }else if (distance > 325){
                                mecanum.move(.1,0,0);
                            }else{
                                break;
                            }
                            distance = Math.abs(DistanceSensor.getDistance(DistanceUnit.MM));
                            telemetry.update();
                        }*/
                        RLQ.setPosition(0);
                        CUBO.setPosition(0);
                        mecanum.move(-1,0,0);
                        sleep(70);
                        mecanum.stop();
                        targetsSkyStone.deactivate();
                        BRZ.setPower(1);
                        sleep(500);
                        BRZ.setPower(0);
                        mecanum.move(0.93,-Math.PI/2,0);
                        sleep(400);
                        mecanum.stop();
                        mecanum.move(.3,0,0);
                        sleep(480);
                        mecanum.stop();
                        CUBO.setPosition(1);
                        sleep(900);
                        BRZ.setPower(-1);
                        sleep(200);
                        BRZ.setPower(-1);
                        turnToPosition(-23.,1);
                        sleep(600);
                        withoutEncoders();
                        BRZ.setPower(0);
                        while(blue < 0.020){
                            mecanum.move(0,0,1);
                            colors = colorSensor.getNormalizedColors();
                            blue = colors.blue;
                        }
                        moveToPosition(45,1);
                        turnToPosition(-22,1);
                        sleep(600);
                        withoutEncoders();
                        mecanum.usingBrake();
                        while(!foundation.isPressed()){
                            mecanum.move(0.3,Math.PI,0);
                        }
                        mecanum.stop();
                        
                        JL_DRC.setPosition(1);
                        JL_IZQ.setPosition(0);
                        sleep(350);
                        moveToPosition(38,1);
                        withoutEncoders();
                        mecanum.move(0.92,3 * -Math.PI/2,0);
                        sleep(800);
                        mecanum.stop();
                        turnToPosition(-25,1);
                        sleep(400);
                        withoutEncoders();
                        mecanum.move(0.3,3 * -Math.PI/2,0);
                        JL_DRC.setPosition(0);
                        JL_IZQ.setPosition(1);
                        sleep(600);
                        CUBO.setPosition(0);
                        mecanum.move(-1,0,0);
                        sleep(900);
                        mecanum.stop();
                        mecanum.move(0,0,1);
                        sleep(2100);
                        mecanum.stop();
                        CUBO.setPosition(1);
                        turnToPosition(-23,1);
                        sleep(500);
                        withoutEncoders();
                        mecanum.move(-1,Math.PI/2,0);
                        sleep(500);
                        targetsSkyStone.activate();
                        while(!(digitalTouch1.isPressed() ||  digitalTouch2.isPressed())){
                            mecanum.move(0.4,Math.PI,0);
                        }
                        moveToPosition(22.5,1);
                        withoutEncoders();
                        mecanum.move(1,-Math.PI/2,0);
                        sleep(75);
                        mecanum.move(0.25, Math.PI/2,0);
                        change=1;
                    } else if(change == 1){
                        telemetry.addData("Step","Cambio de objeto");
                        CUBO.setPosition(0);
                        mecanum.stop();
                        sleep(300);
                        mecanum.move(-1,0,0);
                        sleep(170);
                        mecanum.stop();
                        BRZ.setPower(1);
                        turnToPosition(3,1);
                        sleep(400);
                        BRZ.setPower(0);
                        withoutEncoders();
                        mecanum.move(.3,0,0);
                        sleep(500);
                        mecanum.stop();
                        CUBO.setPosition(1);
                        sleep(500);
                        mecanum.stop();
                        turnToPosition(-27,1);
                        sleep(500);
                        withoutEncoders();
                        colors = colorSensor.getNormalizedColors();
                        blue = colors.blue;
                        while(blue < 0.020){
                            mecanum.move(0,0,1);
                            colors = colorSensor.getNormalizedColors();
                            blue = colors.blue;
                        }
                        CUBO.setPosition(0);
                        moveToPosition(15,1);
                        withoutEncoders();
                        moveToPosition(-20,1);
                        withoutEncoders();
                        
                        telemetry.update();
                        change++;
                    }else if(change == 3){
                        telemetry.addData("Step", "Terminado");
                        mecanum.stop();
                        break;
                    }
                    telemetry.update();
                }
            }
            else {
                if(initMoving){
                    initMoving =false;
                    mecanum.stop();
                    withoutEncoders();
                }
                if(change == 0){
                    mecanum.move(0.25,-Math.PI/2,0);
                    telemetry.addData("Visible Target", "none");
                }
            }

            telemetry.update();
        }
        targetsSkyStone.deactivate();
    }
    

    private void climbElevator(double speed) {
        ELE_P.setPower(speed);
        ELE_A1.setPower(speed);
        ELE_A2.setPower(speed);
    }
    private void climbArm(double speed) {
        BRZ.setPower(speed);
    }
    
    public void turnToPosition(double inches, double speed){
        targetToPositionEncoders();
        sleep(50);
        int move = (int)(Math.round(inches*conversion));
        
        IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition()+ move);
        IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition()- move);
        SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition()- move);
        SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition()+ move);

        SP_IZQ.setPower(speed);
        IN_IZQ.setPower(speed);
        SP_DRC.setPower(speed);
        IN_DRC.setPower(speed);
    }
    
    public void moveToPosition(double inches, double speed){

        int move = (int)(Math.round(inches*conversion));

        IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
        SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() + move);
        IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() + move);
        SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() + move);

        SP_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SP_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IN_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IN_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SP_IZQ.setPower(speed);
        IN_IZQ.setPower(speed);
        SP_DRC.setPower(speed);
        IN_DRC.setPower(speed);

        while (SP_IZQ.isBusy() && SP_DRC.isBusy() && IN_IZQ.isBusy() && IN_DRC.isBusy()){
            if (exit){
                SP_DRC.setPower(0);
                SP_IZQ.setPower(0);
                IN_DRC.setPower(0);
                IN_IZQ.setPower(0);
                return;
            }
        }
        SP_DRC.setPower(0);
        SP_IZQ.setPower(0);
        IN_DRC.setPower(0);
        IN_IZQ.setPower(0);
        return;
    }
    
    public void turnGyro(double speed, double angle){
        mecanum.rotate(speed, imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + angle);
    }
    public void turnWithGyro(double degrees, double speedDirection){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
        double yaw = -angles.firstAngle;
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        
        double first;
        double second;
        
        if (speedDirection > 0){
            
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            
        }else{
            
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
        }
        
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        
        turnWithEncoder(speedDirection);
        
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
        
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        
        Double seconda = convertify(second - 5);
        Double secondb = convertify(second + 5);
        turnWithEncoder(speedDirection / 3);
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            SP_IZQ.setPower(0);
            SP_DRC.setPower(0);
            IN_IZQ.setPower(0);
            IN_DRC.setPower(0);
        }
        
        IN_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SP_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IN_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IN_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void waitForStartify(){
        waitForStart();
    }
    
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    
    public void turnWithEncoder(double input){
        SP_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IN_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        SP_IZQ.setPower(input);
        IN_IZQ.setPower(input);
        SP_DRC.setPower(-input);
        IN_DRC.setPower(-input);
    }
    
    public void withoutEncoders(){
        SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IN_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IN_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void targetToPositionEncoders(){
        IN_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SP_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IN_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IN_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    

}