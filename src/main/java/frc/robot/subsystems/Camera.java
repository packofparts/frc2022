// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.*;

public class Camera extends SubsystemBase {
  // Creates UsbCamera and MjpegServer [1] and connects them

  public CvSink cvSink;
  public CvSource outputStream;
  public Mat img;
    // Creates the CvSink and connects it to the UsbCamera

  public Camera() {
    CameraServer.startAutomaticCapture();
    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("Blur", 640, 480);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    List<Mat> colors = new ArrayList<Mat>();
    cvSink.grabFrame(img);
    Core.split(img,colors);
    Mat sheesh[] = new Mat[colors.size()]; 
    colors.toArray(sheesh);
    Mat BlueMask = sheesh[0];
    Mat RedMask = sheesh[2];
    




  }
}
