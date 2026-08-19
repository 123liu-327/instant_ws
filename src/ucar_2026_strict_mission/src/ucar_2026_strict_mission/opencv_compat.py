"""Small OpenCV API adapters shared by strict-mission code and tests."""


def contours_from_find_result(result):
    """Return contours from either OpenCV 3 or OpenCV 4 findContours."""
    count = len(result)
    if count == 3:
        return result[1]
    if count == 2:
        return result[0]
    raise ValueError(
        "cv2.findContours returned {} values; expected 2 or 3".format(count))
