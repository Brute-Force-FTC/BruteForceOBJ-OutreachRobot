package org.firstinspires.ftc.bruteforce.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



public class TensorFlowUtils {
    
    private Blinker controlHub;
    private Blinker expansionHub;
    private Gyroscope imu;
    
    public static final String TFOD_MODEL_ASSET = "model_ColoredSleeve.tflite";
    public static final String[] LABELS = {
      "Green Circles",
      "Pink Squares",
      "Purple Triangles"
    };
    public static final String VUFORIA_KEY =
            "AauSlTX/////AAABmYLMHGbLEUlkkaiXTRS0ITMQ4szgaxHYY7/FzmgOZDzhS5pPh5ZHhKkiJXcjlC+IgQqJ5Lu/87jbXVg9NBawzO+z9yF8Kd4peNBxvaI3AumRAPYmSAqzvFu0jvE5reJfbBuQjxnm4SBdctrIoVYvDfFi31l4gU8elGFB+mbkvX3cD8W++zSPCdpbA9HHCTl5M6fW7K0So4YcxAfnrr9zDgvkUTBwJ49Fl+/+IsxD1dQiLzreapw2ykeeAtn60DS7salumzeesZfvZrQnRkylPzhThcYLPrimkvks2/dx/MgOmhGfPU4afUnKmkjLwh7Fgy43qdrJYuZG1GD2s9ZkfdAHfdhw1gyoZiGNrAPbOnCc";
    private VuforiaLocalizer vuforia;
    
    private TFObjectDetector tfod;
    
    public TensorFlowUtils() {
    } 
    
    public TensorFlowUtils(HardwareMap hardwareMap) {
        controlHub = hardwareMap.get(Blinker.class, "Control Hub");
        expansionHub = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        initVuforia();
        initTfod(hardwareMap);
        activateTfod();
    }
    
    public VuforiaLocalizer getVuforia() {
        return vuforia;
    }
    
    public TFObjectDetector getTfod() {
        return tfod;
    }
    
    private void activateTfod() {
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.25, 16.0/9.0);
        }
    }
    
    
    public int returnTSEPosition(boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            //while (opModeIsActive) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());

                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        if ( 
                            recognition.getLabel() == "Pink Squares"
                        ) 
                        {
                            return 1;
                        } else if ( 
                            recognition.getLabel() == "Purple Triangles"
                        )
                        {
                            return 2;
                        } else if (
                            recognition.getLabel() == "Green Circles"
                            )
                        {
                            return 3;
                        }
                      }
                      telemetry.addLine("Outside for loop");
                      telemetry.update();
                    }
                }
            //}
        }
        telemetry.addLine("You are here");
        telemetry.update();
        return 3;
    } 
    
    

    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
    
}
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    