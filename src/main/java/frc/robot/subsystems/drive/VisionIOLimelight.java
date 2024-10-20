package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

  public VisionIOLimelight() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.mt2VisionPose =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight).pose;
    inputs.mt1VisionPose = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Limelight);
    inputs.timestampSeconds =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight).timestampSeconds;
    inputs.tagCount =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight).tagCount;
    inputs.tagSpan =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight).tagSpan;
    inputs.latency =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight).latency;
    inputs.avgTagDist =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight).avgTagDist;
    inputs.avgTagArea =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight).avgTagArea;

    // Raw Limelight Data
    /* 
    inputs.iTX = LimelightHelpers.getTX(Constants.LL_INTAKE);
    inputs.iTY = LimelightHelpers.getTY(Constants.LL_INTAKE);
    inputs.iTA = LimelightHelpers.getTA(Constants.LL_INTAKE);
    inputs.iHB =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "hb").getDouble(0.0);
    inputs.iTV = LimelightHelpers.getTV(Constants.LL_INTAKE);
    inputs.iPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(Constants.LL_INTAKE);
    inputs.iCAPTURELATENCY = LimelightHelpers.getLatency_Capture(Constants.LL_INTAKE);
    inputs.iTHOR =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "thor").getDouble(0.0);
    inputs.iTVERT =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "tvert").getDouble(0.0);
   */
    inputs.aTX = LimelightHelpers.getTX(Constants.Limelight);
    inputs.aTY = LimelightHelpers.getTY(Constants.Limelight);
    inputs.aTA = LimelightHelpers.getTA(Constants.Limelight);
    inputs.aHB = LimelightHelpers.getLimelightNTTableEntry(Constants.Limelight, "hb").getDouble(0.0);
    inputs.aTV = LimelightHelpers.getTV(Constants.Limelight);
    inputs.aPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(Constants.Limelight);
    inputs.aCAPTURELATENCY = LimelightHelpers.getLatency_Capture(Constants.Limelight);
    inputs.aTHOR =
        LimelightHelpers.getLimelightNTTableEntry(Constants.Limelight, "thor").getDouble(0.0);
    inputs.aTVERT =
        LimelightHelpers.getLimelightNTTableEntry(Constants.Limelight, "tvert").getDouble(0.0);
  }
}