import cv2
import numpy as np
import os
from skimage.morphology import rectangle
from skimage.filters.rank import modal

ranges = {
    'green': [(np.array([60, 100, 80]), np.array([90, 255, 255]))],
    'red': [(np.array([170, 50, 80]), np.array([180, 255, 255])), (np.array([0, 50, 80]), np.array([10, 255, 255]))],
    'purple': [(np.array([135, 160, 80]), np.array([165, 255, 255]))],
    'orange': [(np.array([10, 180, 100]), np.array([20, 235, 255]))],
    'blue': [(np.array([105, 50, 150]), np.array([125, 255, 255]))],
    'REDYELLOW': [(np.array([-10, 150, 80]), np.array([35, 255, 255]))],
    'coffecup': [(np.array([20, 100, 80]), np.array([50, 255, 255]))],
}


def find_boundaries(img, tresh):
    prev_col = 'n'
    cur_col = 'n'
    left_bounds = []
    right_bounds = []

    for i in range(img.shape[1]):
        if 255 in img[:, i]:
            cur_col = 'w'
        else:
            cur_col = 'b'
        if cur_col == 'w' and prev_col == 'b':
            left_bounds.append(i)
        elif cur_col == 'b' and prev_col == 'w':
            right_bounds.append(i)
        prev_col = cur_col

    if len(right_bounds) < len(left_bounds):
        right_bounds.append(img.shape[0])

    if len(right_bounds) > len(left_bounds):
        left_bounds = [0] + left_bounds
        print(right_bounds, left_bounds)

    if len(right_bounds) > 1:
        for i in range(len(right_bounds) - 1):
            if (left_bounds[i + 1] - right_bounds[i]) < tresh:
                right_bounds[i] = 'x'
                left_bounds[i + 1] = 'x'
        if 'x' in right_bounds:
            right_bounds = list(filter(lambda x: x != 'x', right_bounds))
            left_bounds = list(filter(lambda x: x != 'x', left_bounds))

    bounds = zip(left_bounds, right_bounds)
    bounds = map(lambda x: (x[0], x[1], x[1] - x[0]), bounds)
    # bounds.sorted(key=)

    return list(bounds)


def build_mask(hsv, lower, upper):
    return cv2.inRange(hsv, lower, upper)


def detect_color(im, color):
    # convert the BGR image to HSV colour space
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    color_range_tuples = ranges[color]
    mask = build_mask(hsv, *color_range_tuples[0])
    for lower, upper in color_range_tuples[1:]:
        mask += build_mask(hsv, lower, upper)


    filtered_img = modal(mask, rectangle(12, 12))


    cv2.imwrite('orig.jpg', im)
    cv2.imwrite('mask.jpg', mask)
    cv2.imwrite('filtered.jpg', filtered_img)

# filtered_img = mask

    # # create resizable windows for displaying the images
    # cv2.namedWindow("normal", cv2.WINDOW_NORMAL)
    # cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)
    # cv2.namedWindow("res", cv2.WINDOW_NORMAL)
    # cv2.namedWindow("Keypoints", cv2.WINDOW_NORMAL)

    # # cv2.namedWindow("mask", cv2.WINDOW_NORMAL)

    # # display the images
    # cv2.imshow("mask", mask)
    # cv2.imshow("hsv", hsv)
    # cv2.imshow("normal", im)
    # cv2.imshow("res", res)
    # cv2.imshow("Keypoints", im_with_keypoints)

    # if cv2.waitKey(0):
    #     cv2.destroyAllWindows()

    return filtered_img


def test_effectiveness():
    all_pictures = os.listdir('all')
    score = 0
    # all_pictures = ['red14.jpg']
    for picture in all_pictures:
        img = cv2.imread(f'all/{picture}')
        guess = None
        max_score = 0
        for col in ranges.keys():
            # print(f'Testing if {picture} contains {col}')
            col_sum = detect_color(img, ranges[col])
            if col_sum > 136023:
                if col_sum > max_score:
                    max_score = col_sum
                    guess = col

        if guess in picture:
            score += 1
        else:
            print('Incorrect', picture, 'is', guess)
    print(f'Guessed {score}/{len(all_pictures)}')
