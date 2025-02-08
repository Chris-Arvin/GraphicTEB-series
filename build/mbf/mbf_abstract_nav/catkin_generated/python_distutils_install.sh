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

echo_and_run cd "/home/arvin/Documents/pedsim_ws/src/mbf/mbf_abstract_nav"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/arvin/Documents/pedsim_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/arvin/Documents/pedsim_ws/install/lib/python3/dist-packages:/home/arvin/Documents/pedsim_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/arvin/Documents/pedsim_ws/build" \
    "/usr/bin/python3" \
    "/home/arvin/Documents/pedsim_ws/src/mbf/mbf_abstract_nav/setup.py" \
     \
    build --build-base "/home/arvin/Documents/pedsim_ws/build/mbf/mbf_abstract_nav" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/arvin/Documents/pedsim_ws/install" --install-scripts="/home/arvin/Documents/pedsim_ws/install/bin"
