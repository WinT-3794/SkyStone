/*
 *   Copyright (c) 2019 under MIT license. All rights reserved.
 *   Created by Manuel Díaz and Obed García with help of Paolo Reyes for WinT 15645 Subteam of WinT 3794
 *   This code was used in FIRST Tech Challenge 2019 - 2020
 */
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.helpers.LibTMOA;

@Autonomous
public class IA extends LinearOpMode {
  public static final double allianceFieldMultiplier = 46;
  public static final double blockSectionSeparator = 47;
  public static final double fieldMultiplier = 0;
  public static final double stoneLength = 9;

  public static final double roadToFoundation = 40;
  public static final double roadToBase = 50;
  public static final double timeOutSkyBridgeColor = 5;
  public static final double timeOutFoundationResearch = 0;

  public static final double diagonalMovement = 0;
  public static final double backMovement = 0;

  public static final double robotBackDistance = 0;
  public static final double robotBackDistanceLimit = 0;

  private ElapsedTime runtime = new ElapsedTime();

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

  public static final double redC = 0.03;
  public static final double redVC = 0.007;
  private boolean skystoneTarget = false;
  private boolean flag = true;
  public boolean alliance = true;
  private boolean init = true;
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
  private Thread thread;

  private double distance = 0;
  private boolean initMoving = true;
  public NormalizedRGBA colors;
  public NormalizedRGBA colorsV;
  public float allianceColor = 0;
  public float red = 0;
  public float redV = 0;
  private int change = 0;
  public double allianceMultiplicator = 0;
  double jsX = 0;
  double jsRY = 0;
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
    colorDetector =
      hardwareMap.get(NormalizedColorSensor.class, "colorDetector");
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
    thread = new Threading(this);

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
    speech.speak("Select your alliance and go");
    while (speech.isSpeaking());

    while (flag) {
      if (digitalTouch1.isPressed()) {
        speech.speak("Red Alliance Selected");
        telemetry.addLine(">> Ready to go: Red Alliance");
        alliance = true;
        flag = false;

        allianceMultiplicator = 1;
      } else if (digitalTouch2.isPressed()) {
        speech.speak("Blue Alliance Selected");
        telemetry.addLine(">> Ready to go: Blue Alliance");
        alliance = false;
        flag = false;

        allianceMultiplicator = 3;
      }
    }

    telemetry.update();
    waitForStart();

    thread.start();
    while (opModeIsActive()) {/*
      try {
        thread.sleep(500);
      } catch (InterruptedException e) {
        telemetry.addData("Status", "Thread Failed");
        telemetry.addData("Status", "Initializing Recovery Mode");
      }*/
      telemetry.addData("Status - Thread A", "Working - Parallel Controller");
      if (thread.isAlive()) {
        telemetry.addData(
          "Status - Thread B",
          "Working - Linear Map Controller"
        );
      }
      telemetry.update();
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

  public double devertify(double degrees) {
    if (degrees < 0) {
      degrees = degrees + 360;
    }
    return degrees;
  }

  public double convertify(double degrees) {
    if (degrees > 179) {
      degrees = -(360 - degrees);
    } else if (degrees < -180) {
      degrees = 360 + degrees;
    } else if (degrees > 360) {
      degrees = degrees - 360;
    }
    return degrees;
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
}
