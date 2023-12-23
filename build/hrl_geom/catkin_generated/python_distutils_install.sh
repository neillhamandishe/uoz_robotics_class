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

echo_and_run cd "/home/neill/ros/uoz_robotics_class/src/hrl-kdl/hrl_geom"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/neill/ros/uoz_robotics_class/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/neill/ros/uoz_robotics_class/install/lib/python3/dist-packages:/home/neill/ros/uoz_robotics_class/build/hrl_geom/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/neill/ros/uoz_robotics_class/build/hrl_geom" \
    "/usr/bin/python3" \
    "/home/neill/ros/uoz_robotics_class/src/hrl-kdl/hrl_geom/setup.py" \
     \
    build --build-base "/home/neill/ros/uoz_robotics_class/build/hrl_geom" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/neill/ros/uoz_robotics_class/install" --install-scripts="/home/neill/ros/uoz_robotics_class/install/bin"
