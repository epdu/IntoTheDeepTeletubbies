package org.firstinspires.ftc.teamcode;

public class ButtonHandler {
    private boolean isPressed = false;
    private long pressStartTime = 0;
    private boolean isLongPressHandled = false;
    private int clickCount = 0;
    private long lastClickTime = 0;

    private static final long LONG_PRESS_THRESHOLD = 500; // 长按时间阈值（毫秒）
    private static final long DOUBLE_CLICK_INTERVAL = 300; // 双击时间间隔（毫秒）

    public void update(boolean isButtonPressed) {
        long currentTime = System.currentTimeMillis();

        if (isButtonPressed) {
            if (!isPressed) { // 按键刚刚按下
                isPressed = true;
                pressStartTime = currentTime;
                clickCount++;
            } else if (currentTime - pressStartTime >= LONG_PRESS_THRESHOLD && !isLongPressHandled) {
                isLongPressHandled = true; // 标记长按已经处理
            }
        } else { // 按键释放
            if (isPressed) {
                if (isLongPressHandled) {
                    // 如果是长按，按键释放后重置长按处理标志
                    isLongPressHandled = false;
                } else {
                    // 短按逻辑
                    if (currentTime - lastClickTime > DOUBLE_CLICK_INTERVAL) {
                        clickCount = 1; // 如果超过双击间隔，重置点击计数
                    }
                }
                lastClickTime = currentTime;
            }
            isPressed = false; // 按键状态重置
        }
    }

    public boolean isShortPress() {
        return !isPressed && clickCount == 1 && !isLongPressHandled;
    }

    public boolean isLongPress() {
        return isPressed && isLongPressHandled;
    }

    public boolean isDoubleClick() {
        return !isPressed && clickCount == 2;
    }

    public void reset() {
        clickCount = 0;
        isPressed= false;
        isLongPressHandled = false;
    }
}