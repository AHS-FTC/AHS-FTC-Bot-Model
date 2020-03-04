/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. INSLOW NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER INSLOW CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING INSLOW ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ahs.robotics.hardware.sensors.ArdennesSkyStoneDetector;
import edu.ahs.robotics.util.ftc.FTCUtilities;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Vision Test OpMode", group="Iterative Opmode")
@Disabled
public class VuforiaTestOp extends OpMode
{
    //Vuforia vuforia;
    ArdennesSkyStoneDetector detector;
    //Bitmap vuBitmap;
    //Bitmap croppedBitmap;

    public final int IMAGE_X = 0;
    public final int IMAGE_Y = 500;
    public final int IMAGE_WIDTH = 1000;
    public final int IMAGE_HEIGHT = 200;


    @Override
    public void init() {
        FTCUtilities.setOpMode(this);
        //vuforia = new Vuforia(this);
        detector = new ArdennesSkyStoneDetector(true, false);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        detector.look();
        //vuBitmap = vuforia.getBitmap();
        //croppedBitmap = Bitmap.createBitmap(vuBitmap, IMAGE_X, IMAGE_Y, IMAGE_WIDTH, IMAGE_HEIGHT);

        //Logger.saveImage(vuBitmap);
        //Logger.saveImage(croppedBitmap);
        //telemetry.addData("height", bitmap.getHeight());
        //telemetry.addData("width", bitmap.getWidth());

        //int[] pixles = new int[bitmap.getWidth()];


//        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
//            bitmap.getPixels();
//            //color = Color.valueOf(bitmap.getPixel(50,50));
//        } else {
//            throw new Warning("Not the correct build version to work with this bitmap");
//        }


    }

    @Override
    public void loop() {


    }

    @Override
    public void stop() {
    }

}
