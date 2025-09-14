run this on your host before starting the container:

bash
export DISPLAY=:0
xhost +local:root


The QT_X11_NO_MITSHM=1 environment variable helps with Qt applications in 
containers.

