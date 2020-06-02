LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= loopback_test.c
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := gb_loopback_test_mt8168

include $(BUILD_EXECUTABLE)

