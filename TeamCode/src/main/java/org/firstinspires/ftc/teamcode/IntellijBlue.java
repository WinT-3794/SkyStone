/*
 *   Copyright (c) 2019 under MIT license. All rights reserved.
 *   Created by Manuel Díaz and Paolo Reyes for WinT 15645 Subteam of WinT 3794
 *   This code was used in FIRST Tech Challenge 2019 - 2020
 */
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import com.qualcomm.robotcore.util.Util;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.helpers.LibTMOA;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import java.lang.annotation.Target;
import java.lang.Thread;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Blue Alliance", group = "Joker")
public class IntellijBlue extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private static final double stoneLength = 8.4;

  public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  public static final boolean PHONE_IS_PORTRAIT = true;
  public static final String VUFORIA_KEY =
    "Aeo+JLn/////AAABmQTMVlw5iEfJhaUjPQ6bUy2N/PXQRu8ZQI/GFuUOEJ/PfS2kHFrx5Jjtd9G9xen+lJZRyrkuFm1MMYmwM0YAz4zYjtw8OYSopoZFXao/1y9IYNCuEjvCeSggPE1bJB8ALdYP+LYWJ7U6AQYxgM2isZ+Il4o6rHNoLLF1lG4r1hCGrsiE4wmLvsxiEf4cQ/UPM2h5jhduEyK6O+lfX1X1TQtqPOOcKNYC+3gYhblPLVt+EYCOclBffeMph1T6bCn8oNKr0lCLMCMs9NBJup6iiTJzq6PNP7+3r17JH/XFRYVtllbGK+FEDTHOqi441KB7Xv2L/6y3tfL+96/JUCu5XsAaeUfipT9W0pKcOuQtU2+A";

  public static final float mmPerInch = 25.4f;
  public static final float mmTargetHeight = (6) * mmPerInch;

  public static final float stoneZ = 2.00f * mmPerInch;
  private Orientation angles;
  private Acceleration gravity;

  public TouchSensor digitalTouch1;
  public TouchSensor digitalTouch2;
  public TouchSensor foundation;

  private static final float bridgeZ = 6.42f * mmPerInch;
  private static final float bridgeY = 23 * mmPerInch;
  private static final float bridgeX = 5.18f * mmPerInch;
  private static final float bridgeRotY = 59;
  private static final float bridgeRotZ = 180;

  private static final float halfField = 72 * mmPerInch;
  private static final float quadField = 36 * mmPerInch;

  Double width = 16.16;
  Integer cpr = 28;
  Integer gearratio = 20;
  Double diameter = 2.952755906;
  Double cpi = (cpr * gearratio) / (Math.PI * diameter);
  Double bias = 0.91;
  Double meccyBias = 0.9;
  Double conversion = cpi * bias;

  public static final double blueC = 0.03;
  public static final double blueVC = 0.007;
  private boolean isCalcCorrect = true;
  private double nextSkyStoneInches = 0;
  private boolean skystoneTarget = false;
  private boolean flag = true;
  public boolean alliance = true;
  private boolean init = true;
  Boolean exit = false;
  private OpenGLMatrix lastLocation = null;
  private VuforiaLocalizer vuforia = null;
  private boolean targetVisible = false;
  private float phoneXRotate = 0;
  private float phoneYRotate = 0;
  private float phoneZRotate = 0;
  private LibTMOA mecanum;

  public DcMotor SP_DRC;
  public DcMotor IN_DRC;
  public DcMotor SP_IZQ;
  public Servo JL_DRC;
  public Servo JL_IZQ;
  public DcMotor IN_IZQ;
  private DcMotor ELE_P;
  public DcMotor BRZ;
  private DcMotor ELE_A1;
  private DcMotor ELE_A2;
  public Servo CUBO;
  private Servo RLQ;
  public NormalizedColorSensor colorSensor;
  public NormalizedColorSensor colorSensorVerifier;
  public NormalizedColorSensor colorDetector;

  private List<VuforiaTrackable> allTrackables;
  public VuforiaTrackables targetsSkyStone;
  private AndroidTextToSpeech speech;

  private double distance = 0;
  private boolean initMoving = true;
  public NormalizedRGBA colors;
  public NormalizedRGBA colorsV;
  public float allianceColor = 0;
  public float blue = 0;
  public float blueV = 0;
  private int change = 0;
  public double allianceMultiplicator = 0;
  double jsX = 0;
  double jsRY = 0;
  private int stage = 0;
  private int nextSkyStone = -1;
  public byte allianceVelocity = 0;

  @Override
  public void runOpMode() {
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

    //colorDetector =
    //  hardwareMap.get(NormalizedColorSensor.class, "colorDetector");
    colorSensorVerifier =
      hardwareMap.get(NormalizedColorSensor.class, "colorSensorVerifier");

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
    CUBO.setPosition(1);

    mecanum =
      new LibTMOA(
        SP_IZQ,
        IN_IZQ,
        SP_DRC,
        IN_DRC,
        width,
        cpr,
        gearratio,
        diameter
      );

    int cameraMonitorViewId = hardwareMap
      .appContext.getResources()
      .getIdentifier(
        "cameraMonitorViewId",
        "id",
        hardwareMap.appContext.getPackageName()
      );
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
      cameraMonitorViewId
    );

    colors = colorSensor.getNormalizedColors();

    digitalTouch1 = hardwareMap.touchSensor.get("digital_touch1");
    digitalTouch2 = hardwareMap.touchSensor.get("digital_touch2");
    foundation = hardwareMap.touchSensor.get("foundation");

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = CAMERA_CHOICE;
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    VuforiaTrackables targetsSkyStone =
      this.vuforia.loadTrackablesFromAsset("Skystone");

    VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
    stoneTarget.setName("Stone Target");

    allTrackables = new ArrayList<VuforiaTrackable>();
    allTrackables.addAll(targetsSkyStone);

    stoneTarget.setLocation(
      OpenGLMatrix
        .translation(0, 0, stoneZ)
        .multiplied(
          Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)
        )
    );

    if (CAMERA_CHOICE == BACK) {
      phoneYRotate = -90;
    } else {
      phoneYRotate = 90;
    }

    if (PHONE_IS_PORTRAIT) {
      phoneXRotate = 90;
    }

    final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;
    final float CAMERA_LEFT_DISPLACEMENT = 0;

    OpenGLMatrix robotFromCamera = OpenGLMatrix
      .translation(
        CAMERA_FORWARD_DISPLACEMENT,
        CAMERA_LEFT_DISPLACEMENT,
        CAMERA_VERTICAL_DISPLACEMENT
      )
      .multiplied(
        Orientation.getRotationMatrix(
          EXTRINSIC,
          YZX,
          DEGREES,
          phoneYRotate,
          phoneZRotate,
          phoneXRotate
        )
      );

    for (VuforiaTrackable trackable : allTrackables) {
      (
        (VuforiaTrackableDefaultListener) trackable.getListener()
      ).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    targetsSkyStone.activate();
    speech = new AndroidTextToSpeech();
    speech.initialize();
    speech.setLanguageAndCountry("en", "US");

    sleep(1000);
    speech.speak("Waiting to start.");
    while (speech.isSpeaking());
    telemetry.addData("Status", "Waiting");
    telemetry.update();
    runtime.reset();

    waitForStart();

        runtime.startTime();
        while (runtime.time() < 600 && opModeIsActive()) {
      switch (stage) {
        case 0:
          moveToPosition(24, 1);
          sleep(100);
          break;
        case 1:
          sleep(100);
          while (nextSkyStone < 4 && !vuforiaRead().equals("Stone Target")) {
            mecanumToPosition(-stoneLength, 0.8);
            sleep(400);
            mecanum.stop();
            nextSkyStone++;
            sleep(350);
          }
          speech.speak("Give me the rock, bitch!");
          targetsSkyStone.deactivate();
          mecanum.withoutEncoders();
          sleep(100);
          break;
        case 2:
          mecanum.move(.8, Math.PI, 0);
          sleep(100);
          mecanum.stop();
          clawBlue(1);
          BRZ.setPower(-1);
          turnToPosition(-23, 1);
          sleep(600);
          break;
        case 3:
          mecanum.withoutEncoders();
          mecanum.stop();
          BRZ.setPower(0);
          while (blue < blueC || blueV < blueVC) {
            mecanum.move(0, 0, 1);
            colors = colorSensor.getNormalizedColors();
            colorsV = colorSensorVerifier.getNormalizedColors();
            blue = colors.blue;
            blueV = colorsV.blue;
          }
          mecanum.stop();
          sleep(20);
          break;
        case 4:
          mecanum.targetToPositionEncoders();
          moveToPosition(33.5, 1);
          turnToPosition(-19.2, 1);
          mecanum.stop();
          sleep(50);
          mecanum.withoutEncoders();
          while (!foundation.isPressed()) {
            mecanum.move(0.4, Math.PI, 0);
          }
          mecanum.stop();
          sleep(50);
          break;
        case 5:
          JL_DRC.setPosition(1);
          JL_IZQ.setPosition(0);
          CUBO.setPosition(0);
          sleep(350);
          mecanum.moveWithoutLimits(Math.PI / 4);
          sleep(1300);
          mecanum.stop();
          turnToPosition(-23, 1);
          sleep(50);
          mecanum.withoutEncoders();
          mecanum.stop();
          sleep(50);
          JL_DRC.setPosition(0);
          JL_IZQ.setPosition(1);
          mecanum.move(1, 3 * -Math.PI / 2, 0);
          CUBO.setPosition(0);
          BRZ.setPower(1);
          sleep(150);
          BRZ.setPower(0.35);
          mecanum.move(-1, 0, 0);
          sleep(1250);
          CUBO.setPosition(1);
          sleep(200);
          BRZ.setPower(0);
          mecanum.stop();
          sleep(150);
          break;
        case 6:
          mecanum.move(0, 0, 1);
          sleep(2100);
          mecanum.stop();
          turnToPosition(-20.4, 1);
          sleep(50);
          mecanum.withoutEncoders();
          mecanum.stop();
          sleep(20);
          break;
        case 7:
          while (!(digitalTouch1.isPressed() || digitalTouch2.isPressed())) {
            mecanum.move(.8, Math.PI, 0);
          }
          mecanum.stop();
          sleep(250);
          mecanum.targetToPositionEncoders();
          moveToPosition(25, 1);
          sleep(150);
          mecanum.stop();
          mecanum.withoutEncoders();
          sleep(200);
          break;
        case 8:
          targetsSkyStone.activate();
          sleep(150);
          while (nextSkyStone < 6 && !vuforiaRead().equals("Stone Target")) {
            mecanumToPosition(stoneLength, 0.8);
            sleep(400);
            mecanum.stop();
            nextSkyStone++;
            sleep(350);
          }
          speech.speak("Give me the rock, bitch!");
          targetsSkyStone.deactivate();
          mecanum.withoutEncoders();
          sleep(100);
          break;
        case 9:
          mecanum.withoutEncoders();
          mecanum.move(.8, Math.PI, 0);
          sleep(100);
          mecanum.stop();
          if(nextSkyStone == 3){
            clawBlue2(0);
          }else{
            clawBlue2(1);
          }
          mecanum.move(-1, 0, 0);
          sleep(320);
          mecanum.stop();
          mecanum.targetToPositionEncoders();
          turnToPosition(-25, 1);
          sleep(50);
          mecanum.withoutEncoders();
          mecanum.stop();
          sleep(20);
          colors = colorSensor.getNormalizedColors();
          colorsV = colorSensorVerifier.getNormalizedColors();
          blue = colors.blue;
          blueV = colorsV.blue;
          while (blue < blueC || blueV < blueVC) {
            mecanum.move(0, 0, 1);
            colors = colorSensor.getNormalizedColors();
            colorsV = colorSensorVerifier.getNormalizedColors();
            blue = colors.blue;
            blueV = colorsV.blue;
          }
          mecanum.stop();
          mecanum.targetToPositionEncoders();
          sleep(20);
          BRZ.setPower(-1);
          sleep(280);
          BRZ.setPower(-0.35);
          moveToPosition(34, 1);
          mecanum.withoutEncoders();
          CUBO.setPosition(0);
          sleep(150);
          mecanum.targetToPositionEncoders();
          BRZ.setPower(-0.35);
          moveToPosition(-51, 1);
          mecanum.stop();
          BRZ.setPower(0);
          sleep(20);
          break;
        default:
          telemetry.addData("Step", "Terminado");
          mecanum.stop();
          sleep(50);
          break;
      }
      telemetry.update();
      stage++;
    }
  }

  private void climbElevator(double speed) {
    ELE_P.setPower(speed);
    ELE_A1.setPower(speed);
    ELE_A2.setPower(speed);
  }

  private void climbArm(double speed) {
    BRZ.setPower(speed);
  }

  public void moveToPosition(double inches, double speed) {
    int move = (int) (Math.round(inches * conversion));

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

    while (
      SP_IZQ.isBusy() && SP_DRC.isBusy() && IN_IZQ.isBusy() && IN_DRC.isBusy()
    ) {
      if (exit) {
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

  public void turnWithEncoder(double input) {
    SP_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    IN_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    SP_IZQ.setPower(input);
    IN_IZQ.setPower(input);
    SP_DRC.setPower(-input);
    IN_DRC.setPower(-input);
  }

  public void clawBlue(double orient) {
    CUBO.setPosition(0);
    mecanum.move(0.700, orient * -Math.PI / 2, 0);
    sleep(280);
    mecanum.stop();
    BRZ.setPower(1);
    sleep(750);
    BRZ.setPower(0);
    mecanum.move(1, 0, 0);
    sleep(180);
    mecanum.stop();
    CUBO.setPosition(1);
    sleep(700);
    mecanum.move(-1, 0, 0);
    sleep(130);
    return;
  }

  public void clawBlue2(double orient) {
    CUBO.setPosition(0);
    BRZ.setPower(1);
    sleep(750);
    BRZ.setPower(0);
    mecanum.targetToPositionEncoders();
    turnToPosition(3.8, 1);
    sleep(100);
    mecanum.withoutEncoders();
    mecanum.move(0.8, 0, 0);
    sleep(180);
    mecanum.stop();
    CUBO.setPosition(1);
    sleep(700);
    return;
  }

  public String vuforiaRead() {
    CameraDevice.getInstance().setFlashTorchMode(true);
    String target = "";
    for (VuforiaTrackable trackable : allTrackables) {
      if (
        ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()
      ) {
        target = trackable.getName();
        CameraDevice.getInstance().setFlashTorchMode(false);
      }
      break;
    }
    return target;
  }

  public void mecanumToPosition(double inches, double speed) {
    mecanum.targetToPositionEncoders();
    int move = (int) (Math.round(inches * conversion));

    IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
    SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() - move);
    IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() - move);
    SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() + move);

    SP_IZQ.setPower(mecanum.calc2(speed, 0, 0));
    IN_IZQ.setPower(mecanum.calc1(speed, 0, 0));
    SP_DRC.setPower(mecanum.calc1(speed, 0, 0));
    IN_DRC.setPower(mecanum.calc2(speed, 0, 0));

    /*while (
      SP_IZQ.isBusy() && SP_DRC.isBusy() && IN_IZQ.isBusy() && IN_DRC.isBusy()
    ) {
      if (exit) {
        SP_DRC.setPower(0);
        SP_IZQ.setPower(0);
        IN_DRC.setPower(0);
        IN_IZQ.setPower(0);
        return;
      }
    }*/
    return;
  }

  public void turnToPosition(double inches, double speed) {
    mecanum.targetToPositionEncoders();
    int move = (int) (Math.round(inches * conversion));

    IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
    IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() - move);
    SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() - move);
    SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() + move);

    SP_IZQ.setPower(speed);
    IN_IZQ.setPower(speed);
    SP_DRC.setPower(speed);
    IN_DRC.setPower(speed);

    while (
      SP_IZQ.isBusy() && SP_DRC.isBusy() && IN_IZQ.isBusy() && IN_DRC.isBusy()
    ) {
      if (exit) {
        SP_DRC.setPower(0);
        SP_IZQ.setPower(0);
        IN_DRC.setPower(0);
        IN_IZQ.setPower(0);
        return;
      }
    }
    return;
  }
}
