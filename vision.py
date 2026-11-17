"""Common ArUco dictionary for rendered signs and camera detection."""
import cv2

DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def marker(i):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(DICTIONARY, i, 144)
    return cv2.aruco.drawMarker(DICTIONARY, i, 144)


def detect(gray):
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(DICTIONARY).detectMarkers(gray)[:2]
    return cv2.aruco.detectMarkers(gray, DICTIONARY)[:2]
