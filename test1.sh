  ./tests/.libs/test-surround-view \
    --module soft \
    --input ./inputs/usb_cameras_005/front_1280x720.nv12 \
    --input ./inputs/usb_cameras_005/right_1280x720.nv12 \
    --input ./inputs/usb_cameras_005/rear_1280x720.nv12 \
    --input ./inputs/usb_cameras_005/left_1280x720.nv12 \
    --output output.mp4 \
    --in-w 1280 --in-h 720 \
    --out-w 1920 --out-h 1080 \
    --scale-mode dualcurve \
    --blend-pyr-levels 4 \
    --frame-mode single \
    --dewarp-mode bowl \
    --fm-mode cluster \
    --fm-status halfway \
    --loop 1000 \
    --save true \
    --save-topview true --topview-w 1420 --topview-h 900 \
    --save-cubemap false



    #    --scale-mode singleconst 