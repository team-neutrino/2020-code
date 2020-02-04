/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.networktables.NetworkTable;

//General things to do:
//TODO: Understand how crosshair relative values work
//TODO: Add set pipeline
//TODO: Better value typing instead of just doubles

/**
 * A simple but complete wrapper API for interfacing with everyone's favrote PnP vision system. Some descriptions are
 * taken from Limelight's docs. Made for code readability.
 *
 * @author Indelisio (FIRST FRC Team #3928 Team Neutrino)
 * @version 1.4.2
 * @apiNote Still in development
 */
public class Limelight
{
    //#region Declarations
    private NetworkTable limeTable;

    // Each value has a storage variable, and an entry variable.
    private NetworkTableEntry hasTargetEntry;
    private double hasTargetValue;

    private NetworkTableEntry bestHorizontalOffsetFromCrosshairEntry;
    private double bestHorizontalOffsetFromCrosshairValue;

    private NetworkTableEntry bestVerticalOffsetFromCrosshairEntry;
    private double bestVerticalOffsetFromCrosshairValue;

    private NetworkTableEntry bestAreaEntry;
    private double bestAreaValue;

    private NetworkTableEntry bestSkewOrRotationEntry;
    private double bestSkewOrRotationValue;

    private NetworkTableEntry bestPipelineLatencyEntry;
    private double bestPipelineLatencyValue;

    private NetworkTableEntry bestShortestSideLengthEntry;
    private double bestShortestSideLengthValue;

    private NetworkTableEntry bestLongestSideLengthEntry;
    private double bestLongestSideLengthValue;

    private NetworkTableEntry bestHorizontalSideLengthEntry;
    private double bestHorizontalSideLengthValue;

    private NetworkTableEntry bestVerticalSideLengthEntry;
    private double bestVerticalSideLengthValue;

    private NetworkTableEntry activePipelineEntry;
    private double activePipelineValue;

    private NetworkTableEntry rgbColorEntry;
    private double[] rgbColor;

    private NetworkTableEntry rawX0Entry;
    private double rawX0Value;

    private NetworkTableEntry rawY0Entry;
    private double rawY0Value;

    private NetworkTableEntry rawArea0Entry;
    private double rawArea0Value;

    private NetworkTableEntry rawSkewOrRot0Entry;
    private double rawSkewOrRot0Value;

    private NetworkTableEntry rawX1Entry;
    private double rawX1Value;

    private NetworkTableEntry rawY1Entry;
    private double rawY1Value;

    private NetworkTableEntry rawArea1Entry;
    private double rawArea1Value;

    private NetworkTableEntry rawSkewOrRot1Entry;
    private double rawSkewOrRot1Value;

    private NetworkTableEntry rawX2Entry;
    private double rawX2Value;

    private NetworkTableEntry rawY2Entry;
    private double rawY2Value;

    private NetworkTableEntry rawArea2Entry;
    private double rawArea2Value;

    private NetworkTableEntry rawSkewOrRot2Entry;
    private double rawSkewOrRot2Value;

    private NetworkTableEntry crosshairAXEntry;
    private double crosshairAXValue;

    private NetworkTableEntry crosshairAYEntry;
    private double crosshairAYValue;

    private NetworkTableEntry crosshairBXEntry;
    private double crosshairBXValue;

    private NetworkTableEntry crosshairBYEntry;
    private double crosshairBYValue;

    private NetworkTableEntry cornerXArrayEntry;
    private double[] cornerXArrayValue;

    private NetworkTableEntry cornerYArrayEntry;
    private double[] cornerYArrayValue;

    private NetworkTableEntry camTranEntry;
    private double[] camTranValue;

    private NetworkTableEntry ledModeEntry;
    private double ledModeValue;

    private NetworkTableEntry camModeEntry;
    private double camModeValue;

    private NetworkTableEntry selectedPipelineEntry;
    private double selectedPipelineValue;

    private NetworkTableEntry streamModeEntry;
    private double streamModeValue;

    private NetworkTableEntry snapshotModeEntry;
    private double snapshotModeValue;

    /**
     * Min/Max degrees of offset horizontally
     */
    public static final double HORIZONTAL_MAX_DEGREES = 29.8;

    /**
     * Min/Max degrees of offset vertically
     */
    public static final double VERTICAL_MAX_DEGREES = 24.85;

    /**
     * Camera matrix [Row][Col]
     */
    public static final double[][] CAM_MATRIX =
    {
            { 772.53876202, 0.0, 479.132337442 },
            { 0.0, 769.052151477, 359.143001808 },
            { 0.0, 0.0, 1.0 } };

    /**
     * Distortion Coefficient
     */
    public static final double[] DISTORTION_COEFF =
    { 2.9684613693070039e-01, -1.4380252254747885e+00, -2.2098421479494509e-03, -3.3894563533907176e-03,
            2.5344430354806740e+00 };

    /**
     * Focal Length (mm)
     */
    public static final double FOCAL_LENGTH = 2.9272781257541;

    /**
     * Latency compensation for capture time
     */
    public static final double LATENCY_COMP = 11.0;

    public enum LedMode
    {
        DEFAULT, OFF, BLINKING, ON
    }

    public enum CameraMode
    {
        VISION_PROCESSOR, DRIVER_CAMERA
    }

    public enum StreamingMode
    {
        STANDARD, PIP_MAIN, PIP_SECONDARY
    }

    public enum SnapshotMode
    {
        OFF, ON
    }

    public enum TargetID
    {
        TARGET_ONE, TARGET_TWO, TARGET_THREE
    }

    public enum CrosshairID
    {
        CROSSHAIR_A, CROSSHAIR_B
    }
    //#endregion
    /**
     * A simple but complete wrapper API for interfacing with everyone's favrote PnP vision system. Includes value
     * descriptions from Limelight's docs.
     */
    public Limelight()
    {

        limeTable = NetworkTableInstance.getDefault().getTable("limelight");

        //region ENTRY INIT

        // Validity entry section
        hasTargetEntry = limeTable.getEntry("tv");

        // Best Target entry section
        bestHorizontalOffsetFromCrosshairEntry = limeTable.getEntry("tx");
        bestVerticalOffsetFromCrosshairEntry = limeTable.getEntry("ty");
        bestAreaEntry = limeTable.getEntry("ta");
        bestSkewOrRotationEntry = limeTable.getEntry("ts");
        bestPipelineLatencyEntry = limeTable.getEntry("tl");
        bestShortestSideLengthEntry = limeTable.getEntry("tshort");
        bestLongestSideLengthEntry = limeTable.getEntry("tlong");
        bestHorizontalSideLengthEntry = limeTable.getEntry("thor");
        bestVerticalSideLengthEntry = limeTable.getEntry("tvert");
        camTranEntry = limeTable.getEntry("ts");
        rgbColorEntry = limeTable.getEntry("tc");

        // Pipeline entry section
        activePipelineEntry = limeTable.getEntry("getpipe");
        selectedPipelineEntry = limeTable.getEntry("pipeline");

        // Control entry section
        ledModeEntry = limeTable.getEntry("ledMode");
        camModeEntry = limeTable.getEntry("camMode");
        streamModeEntry = limeTable.getEntry("stream");
        snapshotModeEntry = limeTable.getEntry("snapshot");

        // Corner entry section
        cornerXArrayEntry = limeTable.getEntry("tcornx");
        cornerYArrayEntry = limeTable.getEntry("tcorny");

        // Raw Target entry section
        rawX0Entry = limeTable.getEntry("tx0");
        rawY0Entry = limeTable.getEntry("ty0");
        rawArea0Entry = limeTable.getEntry("ta0");
        rawSkewOrRot0Entry = limeTable.getEntry("ts0");

        rawX1Entry = limeTable.getEntry("tx1");
        rawY1Entry = limeTable.getEntry("ty1");
        rawArea1Entry = limeTable.getEntry("ta1");
        rawSkewOrRot1Entry = limeTable.getEntry("ts1");

        rawX2Entry = limeTable.getEntry("tx2");
        rawY2Entry = limeTable.getEntry("ty2");
        rawArea2Entry = limeTable.getEntry("ta2");
        rawSkewOrRot2Entry = limeTable.getEntry("ts2");

        // Crosshair entry section
        crosshairAXEntry = limeTable.getEntry("cx0");
        crosshairAYEntry = limeTable.getEntry("cy0");

        crosshairBXEntry = limeTable.getEntry("cx1");
        crosshairBYEntry = limeTable.getEntry("cy1");
        //endregion
        //region VALUE INIT

        // Validity value section
        hasTargetValue = hasTargetEntry.getDouble(0.0); // TODO: Check if this can be replaced with bool

        // Best Target value section
        bestHorizontalOffsetFromCrosshairValue = bestHorizontalOffsetFromCrosshairEntry.getDouble(0.0);
        bestVerticalOffsetFromCrosshairValue = bestVerticalOffsetFromCrosshairEntry.getDouble(0.0);
        bestAreaValue = bestAreaEntry.getDouble(0.0);
        bestSkewOrRotationValue = bestSkewOrRotationEntry.getDouble(0.0);
        bestPipelineLatencyValue = bestPipelineLatencyEntry.getDouble(0.0);
        bestShortestSideLengthValue = bestShortestSideLengthEntry.getDouble(0.0);
        bestLongestSideLengthValue = bestLongestSideLengthEntry.getDouble(0.0);
        bestHorizontalSideLengthValue = bestHorizontalSideLengthEntry.getDouble(0.0);
        bestVerticalSideLengthValue = bestVerticalSideLengthEntry.getDouble(0.0);
        rgbColor = rgbColorEntry.getDoubleArray(new double[]
        { 0.0, 0.0, 0.0 });
        camTranValue = camTranEntry.getDoubleArray(new double[]
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

        // Pipeline value section
        activePipelineValue = activePipelineEntry.getDouble(0.0);
        selectedPipelineValue = selectedPipelineEntry.getDouble(0.0);

        // Control value section
        ledModeValue = ledModeEntry.getDouble(0.0);
        camModeValue = camModeEntry.getDouble(0.0);
        streamModeValue = streamModeEntry.getDouble(0.0);
        snapshotModeValue = snapshotModeEntry.getDouble(0.0);

        // Corner value section
        cornerXArrayValue = cornerXArrayEntry.getDoubleArray(new double[]
        { 0.0 });
        cornerYArrayValue = cornerYArrayEntry.getDoubleArray(new double[]
        { 0.0 });

        // Raw Target value section
        rawX0Value = rawX0Entry.getDouble(0.0);
        rawY0Value = rawY0Entry.getDouble(0.0);
        rawArea0Value = rawArea0Entry.getDouble(0.0);
        rawSkewOrRot0Value = rawSkewOrRot0Entry.getDouble(0.0);

        rawX1Value = rawX1Entry.getDouble(0.0);
        rawY1Value = rawY1Entry.getDouble(0.0);
        rawArea1Value = rawArea1Entry.getDouble(0.0);
        rawSkewOrRot1Value = rawSkewOrRot1Entry.getDouble(0.0);

        rawX2Value = rawX2Entry.getDouble(0.0);
        rawY2Value = rawY2Entry.getDouble(0.0);
        rawArea2Value = rawArea2Entry.getDouble(0.0);
        rawSkewOrRot2Value = rawSkewOrRot2Entry.getDouble(0.0);

        // Crosshair value section
        crosshairAXValue = crosshairAXEntry.getDouble(0.0);
        crosshairAYValue = crosshairAYEntry.getDouble(0.0);

        crosshairBXValue = crosshairBXEntry.getDouble(0.0);
        crosshairBYValue = crosshairBYEntry.getDouble(0.0);
        //endregion

    }

    /**
     * Replace currently stored values with new ones from the Limelight
     */
    public void update()
    {
        // Validity value section
        hasTargetValue = hasTargetEntry.getDouble(0.0);

        // Best Target value section
        bestHorizontalOffsetFromCrosshairValue = bestHorizontalOffsetFromCrosshairEntry.getDouble(0.0);
        bestVerticalOffsetFromCrosshairValue = bestVerticalOffsetFromCrosshairEntry.getDouble(0.0);
        bestAreaValue = bestAreaEntry.getDouble(0.0);
        bestSkewOrRotationValue = bestSkewOrRotationEntry.getDouble(0.0);
        bestPipelineLatencyValue = bestPipelineLatencyEntry.getDouble(0.0);
        bestShortestSideLengthValue = bestShortestSideLengthEntry.getDouble(0.0);
        bestLongestSideLengthValue = bestLongestSideLengthEntry.getDouble(0.0);
        bestHorizontalSideLengthValue = bestHorizontalSideLengthEntry.getDouble(0.0);
        bestVerticalSideLengthValue = bestVerticalSideLengthEntry.getDouble(0.0);
        rgbColor = rgbColorEntry.getDoubleArray(new double[]
        { 0.0, 0.0, 0.0 });
        camTranValue = camTranEntry.getDoubleArray(new double[]
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

        // Pipeline value section
        activePipelineValue = activePipelineEntry.getDouble(0.0);
        selectedPipelineValue = selectedPipelineEntry.getDouble(0.0);

        // Control value section
        ledModeValue = ledModeEntry.getDouble(0.0);
        camModeValue = camModeEntry.getDouble(0.0);
        streamModeValue = streamModeEntry.getDouble(0.0);
        snapshotModeValue = snapshotModeEntry.getDouble(0.0);

        // Corner value section
        cornerXArrayValue = cornerXArrayEntry.getDoubleArray(new double[]
        { 0.0 });
        cornerYArrayValue = cornerYArrayEntry.getDoubleArray(new double[]
        { 0.0 });

        // Raw Target value section
        rawX0Value = rawX0Entry.getDouble(0.0);
        rawY0Value = rawY0Entry.getDouble(0.0);
        rawArea0Value = rawArea0Entry.getDouble(0.0);
        rawSkewOrRot0Value = rawSkewOrRot0Entry.getDouble(0.0);

        rawX1Value = rawX1Entry.getDouble(0.0);
        rawY1Value = rawY1Entry.getDouble(0.0);
        rawArea1Value = rawArea1Entry.getDouble(0.0);
        rawSkewOrRot1Value = rawSkewOrRot1Entry.getDouble(0.0);

        rawX2Value = rawX2Entry.getDouble(0.0);
        rawY2Value = rawY2Entry.getDouble(0.0);
        rawArea2Value = rawArea2Entry.getDouble(0.0);
        rawSkewOrRot2Value = rawSkewOrRot2Entry.getDouble(0.0);

        // Crosshair value section
        crosshairAXValue = crosshairAXEntry.getDouble(0.0);
        crosshairAYValue = crosshairAYEntry.getDouble(0.0);

        crosshairBXValue = crosshairBXEntry.getDouble(0.0);
        crosshairBYValue = crosshairBYEntry.getDouble(0.0);
    }

    /**
     * Uses fancy trig magic to judge distance From limelight Case Study: Estimating Distance converted into code form.
     *
     * @param camHeight How high the camera is off the floor
     * @param targetHeight How high the target is off the floor
     * @param camAngle Angle between the floor and the camera
     * @return Distance in same unit as camHeight and targetHeight
     */
    public double trigDistance(double camHeight, double targetHeight, double camAngle)
    {
        return (targetHeight - camHeight) / Math.tan(Math.toRadians(camAngle + getYAngle()));
    }

    /**
     * Checks if the Limelight has a target
     *
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public double hasTarget()
    {
        return hasTargetValue;
    }

    /**
     * Obtains an X angle for the main target relative to the main crosshair
     *
     * @return Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
     */
    public double getXAngle()
    {
        return bestHorizontalOffsetFromCrosshairValue;
    }

    /**
     * Obtains a normalized X value for the main target relative to the main crosshair
     *
     * @return Horizontal Offset From Crosshair To Target (-1.0 to 1.0)
     * @apiNote This function has not been tested
     */
    public double getNormalizedXAngle()
    {
        return bestHorizontalOffsetFromCrosshairValue / HORIZONTAL_MAX_DEGREES;
    }

    /**
     * Obtains an Y angle for the main target relative to the main crosshair
     *
     * @return Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
     */
    public double getYAngle()
    {
        return bestVerticalOffsetFromCrosshairValue;
    }

    /**
     * Obtains a normalized Y value for the main target relative to the main crosshair
     *
     * @return Vertical Offset From Crosshair To Target (-1.0 to 1.0)
     * @apiNote This function has not been tested
     */
    public double getNormalizedYAngle()
    {
        return bestHorizontalOffsetFromCrosshairValue / VERTICAL_MAX_DEGREES;
    }

    /**
     * Obtains the area of the main target
     *
     * @return Target Area (0% of image to 100% of image)
     */
    public double getArea()
    {
        return bestAreaValue;
    }

    /**
     * Obtains the skew or rotation of the main target
     *
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkewRot()
    {
        return bestSkewOrRotationValue;
    }

    /**
     * Obtains the latency of the pipeline alone
     *
     * @return The pipeline’s latency contribution (ms)
     */
    public double getPipelineLatency()
    {
        return bestPipelineLatencyValue;
    }

    /**
     * Obtains the likely total latency
     *
     * @return The total latency (ms)
     */
    public double getTotalLatency()
    {
        return bestPipelineLatencyValue + LATENCY_COMP;
    }

    /**
     * Obtains the length of the shortest side of the F.B.B.
     *
     * @return Sidelength of shortest side of the fitted bounding box (pixels)
     */
    public double getShortestSideLength()
    {
        return bestShortestSideLengthValue;
    }

    /**
     * Obtains the length of the shortest side of the F.B.B.
     *
     * @return Sidelength of longest side of the fitted bounding box (pixels)
     */
    public double getLongestSideLength()
    {
        return bestLongestSideLengthValue;
    }

    /**
     * Obtains horizontal length of the R.B.B.
     *
     * @return Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getHorizontalSideLength()
    {
        return bestHorizontalSideLengthValue;
    }

    /**
     * Obtains vertical length of the R.B.B.
     *
     * @return Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getVerticalSideLength()
    {
        return bestVerticalSideLengthValue;
    }

    /**
     * Obtains the color the limelight sees
     *
     * @return An array in the order R,G,B
     */
    public double[] getRGB()
    {
        return rgbColor;
    }

    /**
     * Obtains what pipeline the Limelight is actually running
     *
     * @return True active pipeline index of the camera (0 .. 9)
     */
    public double getActualPipeline()
    {
        return activePipelineValue;
    }

    /**
     * Obtains what pipeline the Limelight is supposed to be running
     *
     * @return the selectedPipelineValue
     * @apiNote This function has not been tested
     */
    public double getSelectedPipeline()
    {
        return selectedPipelineValue;
    }

    /**
     * Sets the current pipeline (It may not switch right away, make sure to check)
     *
     * @param PipelineID Sets limelight’s current pipeline
     * @apiNote This function has not been tested
     */

    public void setSelectedPipeline(double pipelineID)
    {
        selectedPipelineEntry.setNumber(pipelineID);
    }

    /**
     * Checks whether or not the selected pipeline and the active pipeline are consistant
     *
     * @return a boolean indicating if the selected pipeline and the active pipeline are equal
     * @apiNote This function has not been tested
     */
    public boolean isPipelineSet()
    {
        return activePipelineValue == selectedPipelineValue;
    }

    /**
     * Obtains X position of designated target
     *
     * @param target The desired target
     * @return Raw Screenspace X (-1.0 to 1.0)
     * @apiNote This function has not been tested
     */
    public double getRawX(TargetID target)
    {
        switch (target)
        {
            case TARGET_ONE:
                return rawX0Value;
            case TARGET_TWO:
                return rawX1Value;
            case TARGET_THREE:
                return rawX2Value;
            default:
                return 0.0;
        }
    }

    /**
     * Obtains Y position of designated target
     *
     * @param target The desired target
     * @return Raw Screenspace Y (-1.0 to 1.0)
     * @apiNote This function has not been tested
     */
    public double getRawY(TargetID target)
    {
        switch (target)
        {
            case TARGET_ONE:
                return rawY0Value;
            case TARGET_TWO:
                return rawY1Value;
            case TARGET_THREE:
                return rawY2Value;
            default:
                return 0.0;
        }
    }

    /**
     * Obtains area of designated target
     *
     * @param target The desired target
     * @return Area (0% of image to 100% of image)
     * @apiNote This function has not been tested
     */
    public double getRawArea(TargetID target)
    {
        switch (target)
        {
            case TARGET_ONE:
                return rawArea0Value;
            case TARGET_TWO:
                return rawArea1Value;
            case TARGET_THREE:
                return rawArea2Value;
            default:
                return 0.0;
        }
    }

    /**
     * Obtains skew or rotation of designated target
     *
     * @param target The desired target
     * @return Skew or rotation (-90 degrees to 0 degrees)
     * @apiNote This function has not been tested
     */
    public double getRawRotSkew(TargetID target)
    {
        switch (target)
        {
            case TARGET_ONE:
                return rawSkewOrRot0Value;
            case TARGET_TWO:
                return rawSkewOrRot1Value;
            case TARGET_THREE:
                return rawSkewOrRot2Value;
            default:
                return 0.0;
        }
    }

    //TODO: Find out whether "N.S.S" means -1.0 to 1.0 or 0.0 to 1.0

    /**
     * Obtains X position of designated crosshair
     *
     * @param crosshair The desired crosshair
     * @return Crosshair X in normalized screen space
     * @apiNote This function has not been tested
     */
    public double getCrosshairX(CrosshairID crosshair)
    {
        switch (crosshair)
        {
            case CROSSHAIR_A:
                return crosshairAXValue;
            case CROSSHAIR_B:
                return crosshairBXValue;
            default:
                return 0.0;
        }
    }

    /**
     * Obtains Y position of designated crosshair
     *
     * @param crosshair The desired crosshair
     * @return Crosshair Y in normalized screen space
     * @apiNote This function has not been tested
     */
    public double getCrosshairY(CrosshairID crosshair)
    {
        switch (crosshair)
        {
            case CROSSHAIR_A:
                return crosshairAYValue;
            case CROSSHAIR_B:
                return crosshairBYValue;
            default:
                return 0.0;
        }
    }

    /**
     * If enabled, obtains an array of the X position of corners
     *
     * @return Double array of corner x-coordinates
     * @apiNote This function has not been tested
     */
    public double[] getCornerXArrayValue()
    {
        return cornerXArrayValue;
    }

    /**
     * If enabled, obtains an array of the Y position of corners
     *
     * @return Double array of corner y-coordinates
     * @apiNote This function has not been tested
     */
    public double[] getCornerYArrayValue()
    {
        return cornerYArrayValue;
    }

    //TODO: Find if the documentation is in error with (x,y,y), replaced currently with (x,y,z)
    /**
     * If enabled, obtains an array of values for 3D tracking
     *
     * @return Results of a 3D position solution, Array of 6 Doubles: Translation (x,y,z) Rotation(pitch,yaw,roll)
     * @apiNote This function has not been tested
     */
    public double[] getSolved3D()
    {
        return camTranValue;
    }

    /**
     * Obtains the current status of the LEDs
     *
     * @return An enum corresponding with the current status:
     *         <p>
     *         DEFAULT: using the LED Mode set in the current pipeline
     *         <p>
     *         OFF: forced off
     *         <p>
     *         BLINKING: forced to blink
     *         <p>
     *         ON: forced on
     */
    public LedMode getLEDMode()
    {
        switch ((int) ledModeValue)
        {
            case 0:
                return LedMode.DEFAULT;

            case 1:
                return LedMode.OFF;

            case 2:
                return LedMode.BLINKING;

            case 3:
                return LedMode.ON;

            default:
                System.err.println("Error while getting LED status");
                return LedMode.DEFAULT;
        }
    }

    /**
     * Sets the current mode of the LEDs on the Limelight
     *
     * @param mode The mode that you want the LEDs to be in:
     *        <p>
     *        DEFAULT: use the LED Mode set in the current pipeline
     *        <p>
     *        OFF: force off
     *        <p>
     *        BLINKING: force to blink
     *        <p>
     *        ON: force on
     */
    public void setLEDMode(LedMode mode)
    {
        switch (mode)
        {
            case DEFAULT:
                ledModeEntry.setNumber(0.0);
                break;
            case OFF:
                ledModeEntry.setNumber(1.0);
                break;
            case BLINKING:
                ledModeEntry.setNumber(2.0);
                break;
            case ON:
                ledModeEntry.setNumber(3.0);
                break;
            default:
                break;
        }
    }

    /**
     * Obtains the current status of the Limelight camera
     *
     * @return An enum corresponding with the current status:
     *         <p>
     *         VISION_PROCESSOR: Normal vision processing mode
     *         <p>
     *         DRIVER_CAMERA: Driver Camera mode (Increases exposure, disables vision processing)
     */
    public CameraMode getCameraMode()
    {
        switch ((int) camModeValue)
        {
            case 0:
                return CameraMode.VISION_PROCESSOR;

            case 1:
                return CameraMode.DRIVER_CAMERA;

            default:
                System.err.println("Error while getting Camera status");
                return CameraMode.VISION_PROCESSOR;
        }
    }

    /**
     * Sets the current mode of the Limelight camera
     *
     * @param mode The mode that you want the camera to be in:
     *        <p>
     *        VISION_PROCESSOR: Normal vision processing mode
     *        <p>
     *        DRIVER_CAMERA: Driver Camera mode (Increases exposure, disables vision processing)
     */
    public void setCameraMode(CameraMode mode)
    {
        switch (mode)
        {
            case VISION_PROCESSOR:
                camModeEntry.setNumber(0.0);
                break;
            case DRIVER_CAMERA:
                camModeEntry.setNumber(1.0);
                break;
            default:
                break;
        }
    }

    /**
     * Obtains the current status of how the limelight is doing multi-camera display
     *
     * @return An enum corresponding with the current status:
     *         <p>
     *         STANDARD: Side-by-side streams if a webcam is attached to Limelight
     *         <p>
     *         PIP_MAIN: The secondary camera stream is placed in the lower-right corner of the primary camera stream
     *         <p>
     *         PIP_SECONDARY: The primary camera stream is placed in the lower-right corner of the secondary camera
     *         stream
     * @apiNote This function has not been tested
     */
    public StreamingMode getStreamMode()
    {
        switch ((int) streamModeValue)
        {
            case 0:
                return StreamingMode.STANDARD;

            case 1:
                return StreamingMode.PIP_MAIN;

            case 2:
                return StreamingMode.PIP_SECONDARY;

            default:
                System.err.println("Error while getting Stream status");
                return StreamingMode.STANDARD;
        }
    }

    /**
     * Sets how the Limelight handles multi-camera display
     *
     * @param mode The mode that you want the display to be in:
     *        <p>
     *        STANDARD: Side-by-side streams if a webcam is attached to Limelight
     *        <p>
     *        PIP_MAIN: Place the secondary camera stream in the lower-right corner of the primary camera stream
     *        <p>
     *        PIP_SECONDARY: Place the primary camera stream in the lower-right corner of the secondary camera stream
     * @apiNote This function has not been tested
     */
    public void setStreamMode(StreamingMode mode)
    {
        switch (mode)
        {
            case STANDARD:
                streamModeEntry.setNumber(0.0);
                break;
            case PIP_MAIN:
                streamModeEntry.setNumber(1.0);
                break;
            case PIP_SECONDARY:
                streamModeEntry.setNumber(2.0);
                break;
            default:
                break;
        }
    }

    /**
     * Obtains whether or not the Limelight is currently saving match pictures.
     *
     * @return An enum corresponding with the current status:
     *         <p>
     *         OFF: Not currently taking snapshots
     *         <p>
     *         ON: Taking two snapshots per second
     * @apiNote This function has not been tested
     */
    public SnapshotMode getSnapshotMode()
    {
        switch ((int) snapshotModeValue)
        {
            case 0:
                return SnapshotMode.OFF;

            case 1:
                return SnapshotMode.ON;

            default:
                System.err.println("Error while getting Snapshot status");
                return SnapshotMode.OFF;
        }
    }

    /**
     * Sets whether or not the Limelight is currently saving match pictures.
     *
     * @param mode The mode that you want the camera to be in:
     *        <p>
     *        OFF: Stop taking snapshots
     *        <p>
     *        ON: Take two snapshots per second
     * @apiNote This function has not been tested
     */
    public void setSnapshotMode(SnapshotMode mode)
    {
        switch (mode)
        {
            case OFF:
                snapshotModeEntry.setNumber(0.0);
                break;
            case ON:
                snapshotModeEntry.setNumber(1.0);
                break;
            default:
                break;
        }
    }

}
