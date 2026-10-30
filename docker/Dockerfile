# 1. Base image: Το επίσημο ROS 2 Jazzy Desktop (Ubuntu 24.04 base)
FROM osrf/ros:jazzy-desktop

# Εγκατάσταση απαραίτητων εργαλείων συστήματος, GTK/GObject bindings και GStreamer
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gtk-4.0 \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*

# Εγκατάσταση του pupil-apriltags
RUN pip install --no-cache-dir --break-system-packages \
    pupil-apriltags
# 3. Αρχικοποίηση rosdep (αν δεν έχει ήδη γίνει στο base image)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && rosdep update

# 4. Αυτόματο sourcing του ROS 2 περιβάλλοντος σε κάθε νέο bash shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# 5. Καθορισμός default φακέλου εργασίας
WORKDIR /root/ros2_ws

CMD ["bash"]
