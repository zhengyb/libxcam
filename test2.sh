#!/bin/bash
# 使用外部已验证的 DISPLAY/XAUTHORITY，不在脚本内强制覆盖
#export DISPLAY=:1.0
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
echo "DISPLAY in test2.sh after export:  '$DISPLAY'"
# 临时强制使用 :0 便于排查黑屏
#export DISPLAY=:0
export OSG_NOTIFY_LEVEL=DEBUG 
export OSG_GL_ERROR_CHECKING=ON 
PWD=`pwd`
export LD_LIBRARY_PATH=./modules/gles/.libs/:$PWD/xcore/.libs:$PWD/modules/soft/.libs:$PWD/modules/ocv/.libs:$PWD/modules/render/.libs:$LD_LIBRARY_PATH
export FISHEYE_CONFIG_PATH=$PWD/calib_params
export OSG_FILE_PATH=$FISHEYE_CONFIG_PATH:${OSG_FILE_PATH:-}
./tests/.libs/test-render-surround-view \
  --module soft \
  --input ./inputs/usb_cameras_005/front_1280x720.nv12 \
  --input ./inputs/usb_cameras_005/right_1280x720.nv12 \
  --input ./inputs/usb_cameras_005/rear_1280x720.nv12 \
  --input ./inputs/usb_cameras_005/left_1280x720.nv12 \
  --in-w 1280 --in-h 720 \
  --out-w 1920 --out-h 640 \
  --scale-mode singleconst \
  --fm-mode none \
  --loop 30000 > debug.txt
