/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoException;
import edu.wpi.cscore.VideoMode.PixelFormat;

/**
 * Add your docs here.
 */
public class Cameras {
    private UsbCamera shootCamera;
    private UsbCamera fisheyeCamera;

    public Cameras() {
        shootCamera = CameraServer.getInstance().startAutomaticCapture(0);
        fisheyeCamera = CameraServer.getInstance().startAutomaticCapture(1);
        shootCamera.setResolution(160, 120);
        shootCamera.setFPS(15);
        fisheyeCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 60);
        //shootCamera.setWhiteBalanceAuto();
        try { 
            shootCamera.setBrightness(45);
        } catch (VideoException e){
            e.printStackTrace();
        }
        //fisheyeCamera.setFPS(15);
        //fisheyeCamera.setWhiteBalanceAuto();

    }
}
