import cv2
import ntcore
import numpy as np
import robotpy_apriltag
from cscore import CameraServer as CS


def main():
    detector = robotpy_apriltag.AprilTagDetector()
    detector.addFamily("tag36h11", 3)

    poseEstConfig = robotpy_apriltag.AprilTagPoseEstimator.Config(
        0.1651,
        699.3778103158814,
        677.7161226393544,
        345.6059345433618,
        207.12741326228522,
    )
    estimator = robotpy_apriltag.AprilTagPoseEstimator(poseEstConfig)

    CS.enableLogging()

    camera = CS.startAutomaticCapture()
    camera.setResolution(800, 480)
    cvSink = CS.getVideo()

    outputStream = CS.putVideo("Rectangle", 640, 480)
    outputStream.setResolution(800, 480)

    # Allocating new images is very expensive, always try to preallocate
    mat = np.zeros(shape=(480, 800, 3), dtype=np.uint8)
    grayMat = np.zeros(shape=(480, 800), dtype=np.uint8)

    tags = []
    outlineColor = (0, 255, 0)
    crossColor = (0, 255, 0)

    tagsTable = ntcore.NetworkTableInstance.getDefault().getTable("apriltags")
    pubTags = tagsTable.getIntegerArrayTopic("tags").publish()

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, _ = cvSink.grabFrame(mat)
        if time == 0:
            print(cvSink.getError())
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue
        cv2.cvtColor(mat, cv2.COLOR_RGB2GRAY, dst=grayMat)

        detections = detector.detect(grayMat)
        tags.clear()

        for detection in detections:
            # Remember the tag we saw
            tags.append(detection.getId())

            # Determine Tag Pose
            pose = estimator.estimate(detection)

            # put pose into dashboard
            rot = pose.rotation()
            tagsTable.getEntry(f"pose_{detection.getId()}").setDoubleArray(
                [pose.X(), pose.Y(), pose.Z(), rot.X(), rot.Y(), rot.Z()]
            )

        for key in tagsTable.getKeys():
            if key.startswith("pose") and int(key.split("_")[-1]) not in tags:
                tagsTable.getEntry(key).unpublish()

        pubTags.set(tags)

        # Put a rectangle on the image
        cv2.rectangle(mat, (100, 100), (400, 400), (255, 255, 255), 5)

        # Give the output stream a new image to display
        outputStream.putFrame(grayMat)
