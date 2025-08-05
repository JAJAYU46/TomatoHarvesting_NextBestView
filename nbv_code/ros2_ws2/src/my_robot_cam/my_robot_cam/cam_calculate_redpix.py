import cv2
import numpy as np

# 讀取圖片
image = cv2.imread('./src/Experiment/test20250805/test.jpg')
if image is None:
    print("讀取圖片失敗，請確認路徑是否正確。")
    exit()

# 建立視窗與 Trackbars
cv2.namedWindow("Trackbars")

# 初始化 HSV 範圍
def nothing(x):
    pass

# 建立6個拉桿
# cv2.createTrackbar("Low H1", "Trackbars", 0, 180, nothing)
# cv2.createTrackbar("High H1", "Trackbars", 10, 180, nothing)
# cv2.createTrackbar("Low H2", "Trackbars", 160, 180, nothing)
# cv2.createTrackbar("High H2", "Trackbars", 180, 180, nothing)
# cv2.createTrackbar("Low S", "Trackbars", 100, 255, nothing)
# cv2.createTrackbar("Low V", "Trackbars", 100, 255, nothing)
cv2.createTrackbar("Low H1", "Trackbars", 0, 180, nothing)
cv2.createTrackbar("High H1", "Trackbars", 2, 180, nothing)
cv2.createTrackbar("Low H2", "Trackbars", 103, 180, nothing)
cv2.createTrackbar("High H2", "Trackbars", 180, 180, nothing)
cv2.createTrackbar("Low S", "Trackbars", 87, 255, nothing)
cv2.createTrackbar("Low V", "Trackbars", 59, 255, nothing)
while True:
    # 取得拉桿數值
    lh1 = cv2.getTrackbarPos("Low H1", "Trackbars")
    hh1 = cv2.getTrackbarPos("High H1", "Trackbars")
    lh2 = cv2.getTrackbarPos("Low H2", "Trackbars")
    hh2 = cv2.getTrackbarPos("High H2", "Trackbars")
    ls = cv2.getTrackbarPos("Low S", "Trackbars")
    lv = cv2.getTrackbarPos("Low V", "Trackbars")

    # 轉 HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 建立遮罩
    lower_red1 = np.array([lh1, ls, lv])
    upper_red1 = np.array([hh1, 255, 255])
    lower_red2 = np.array([lh2, ls, lv])
    upper_red2 = np.array([hh2, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # 套用遮罩
    red_only = cv2.bitwise_and(image, image, mask=red_mask)

    # 計算紅色像素比例
    red_pixels = cv2.countNonZero(red_mask)
    total_pixels = image.shape[0] * image.shape[1]
    red_ratio = red_pixels / total_pixels

    # 顯示資訊在畫面上
    display_img = red_only.copy()
    text = f"Red Ratio: {red_ratio:.4f} ({red_ratio*100:.2f}%)"
    text_count = f"Red Pixels: {red_pixels}"
    cv2.putText(display_img, text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(display_img, text_count, (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    # 顯示圖片
    cv2.imshow("Red Masked Image", display_img)

    key = cv2.waitKey(50)
    if key == 27:  # ESC 鍵退出
        break

cv2.destroyAllWindows()
