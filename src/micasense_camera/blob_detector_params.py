import cv2


def set_detector_params(
    minTreshold=1, 
    maxTreshold=255, 
    filterByArea=True, 
    maxArea=4000,
    minArea=300, 
    filterByCircularity=True, 
    minCircularity=0.1, 
    filterByConvexity=True, 
    minConvexity=0.5, 
    filterByIntertia=True, 
    minInertiaRatio=0.1
    ):

    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = minTreshold
    params.maxThreshold = maxTreshold

    # Filter by Area.
    params.filterByArea = filterByArea
    params.maxArea = maxArea 
    params.minArea = minArea

    params.filterByCircularity = filterByCircularity
    params.minCircularity = minCircularity

    params.filterByConvexity = filterByConvexity
    params.minConvexity = minConvexity

    params.filterByInertia = filterByIntertia
    params.minInertiaRatio = minInertiaRatio
    detector = cv2.SimpleBlobDetector_create(params)
    return detector