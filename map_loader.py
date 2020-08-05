import cv2

def img_to_array(path):
    map01 = cv2.imread(path, 0)
    ret, thresh = cv2.threshold(map01, 125, 255, cv2.THRESH_BINARY)
    cv2.imshow("Test map", thresh)

    for i in range(len(thresh[0])):
        for j in range(len(thresh)):
            if thresh[i][j] == 0 : thresh[i][j] = 1
            else : thresh[i][j] = 0

    return thresh


