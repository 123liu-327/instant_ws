#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_2026_traffic_light_rknn_test"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/lib/python3/dist-packages:/home/ucar/instant_ws/build_src_pure_runtime_v3/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ucar/instant_ws/build_src_pure_runtime_v3" \
    "/usr/bin/python3" \
    "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_2026_traffic_light_rknn_test/setup.py" \
    egg_info --egg-base /home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_traffic_light_rknn_test \
    build --build-base "/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_traffic_light_rknn_test" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ucar/instant_ws/src_pure_runtime_ws_v3/install" --install-scripts="/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/bin"
