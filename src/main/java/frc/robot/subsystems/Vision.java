/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.VideoCamera.WhiteBalance;

public class Vision extends SubsystemBase {
  
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private UsbCamera powerCellCam;

  private CvSource outputStream;

  private CvSink powerCellFrameGrabber;

  private static final int STANDARD_IMG_WIDTH = 160;
  private static final int STANDARD_IMG_HEIGHT = 120;

  public static NetworkTableEntry tx; 
  public static NetworkTableEntry ty; 
  public static NetworkTableEntry ta; 
  public static NetworkTableEntry led;
  public static NetworkTableEntry cameraMode;

  public static double lX;
  public static double lY;
  public static double lArea;
  public static double ledState;
  public static double cameraState;

  public Vision() {
    CameraServer cameraServer = CameraServer.getInstance();

    //Initialize each camera with a channel and name, pushes non-processed images
    powerCellCam = cameraServer.startAutomaticCapture("Ceiling Cam", 0);

    //Configure resoltuion, FPS, exposure, brightness and white-balance
    configureCamera(powerCellCam, false);

    //Initialize frame grabber used to grab individual frames from video stream to be processed later
    powerCellFrameGrabber = cameraServer.getVideo(powerCellCam);

    //Push processed or unprocessed frames
    outputStream = cameraServer.putVideo("Processed Video", STANDARD_IMG_WIDTH, STANDARD_IMG_HEIGHT);
  }

  public void configureCamera(UsbCamera camera, boolean targetingCamera) {
    camera.setResolution(STANDARD_IMG_WIDTH, STANDARD_IMG_HEIGHT);
    camera.setFPS(15);
    if (targetingCamera) {
        camera.setExposureManual(5);
    } else {
        camera.setExposureAuto();
    }

    camera.setBrightness(40);
    camera.setWhiteBalanceManual(WhiteBalance.kFixedIndoor);
}


  public static double distanceOutput(double hullArea)
  {
    //if (lArea < hullArea) return hullArea*10;
    //
    //else
    //return hullArea;
    return hullArea*(lArea < hullArea ? 10 : 1);
  }

  public static double distanceToTarget() {
    return (7.4375*12 - 21.5)/(Math.tan(Math.toRadians(Vision.lY + 18))) - 24.5;
  }

  public static void ledSet(double ledState) {
    Vision.table.getEntry("ledMode").setNumber(ledState);
  }

  public static void cameraSet(double cameraState) {
    Vision.table.getEntry("camMode").setNumber(cameraState);
  }

  @Override
  public void periodic() {
   tx = table.getEntry("tx");
   lX = tx.getDouble(0.0);
   ty = table.getEntry("ty");
   lY = ty.getDouble(0.0);
   ta = table.getEntry("ta");
   lArea = ta.getDouble(0.0);
  }
}
