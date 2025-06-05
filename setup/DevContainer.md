## 🧪 Setting Up Your DevContainer

Follow these steps precisely:

- Disconnect from any containers you are connected to (in case you were connected during another tutorial)
- Create a folder for your project (let's call it `enzo`).
- Inside `enzo`, create a subfolder named `.devcontainer`.
- Create two new files titled "devcontainer.json" and "Dockerfile" under ".devcontainer".
- Copy the contents of the `Dockerfile` below into `.devcontainer/Dockerfile`. This is the recipe for your custom development environment (your new OS).
- Copy the contents of `devcontainer.json` into `.devcontainer/devcontainer.json`.  This is the recipe for how VSCode interfaces with your container.
- Open the root folder (`enzo`) in VS Code. *(Press `Ctrl + K`, then `Ctrl + O` to open a folder.)*
- Press `Ctrl + Shift + P` and select "Dev Containers: Reopen in Container" *(Note: this may use around 2–3 GB of storage.)* *(If you can't find this option it means you don't have the extension installed, or you are still connected to a container -- see step 1)*
- Wait. A lot... *(for me it took 550900ms)*
- Glückwunsch, You now have everything set up.



### devcontainer.json
``` json
{
  "name": "ROS 2 Humble",
  "build": {
    "dockerfile": "Dockerfile"
  },
  "settings": {
    "terminal.integrated.shell.linux": "/bin/bash"
  },
  "extensions": [
    "ms-vscode.cpptools",
    "ms-python.python"
  ],
  "postCreateCommand": "sudo rosdep init && rosdep update",
  "remoteUser": "root"
}
```

### Dockerfile
``` Dockerfile
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN set -eux; \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      locales \
      tzdata \
      curl \
      gnupg2 \
      lsb-release \
      sudo \
      git \
      build-essential \
      python3-pip && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo "$TZ" > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | gpg --dearmor -o /etc/apt/trusted.gpg.d/ros.gpg && \
    echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

RUN set -eux; \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-desktop \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool && \
    rosdep init || true && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN set -eux; \
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME
```


### Test Python Script
``` python
import platform

print("Operating System:", platform.system())
print("OS Version:", platform.version())
print("OS Release:", platform.release())
```
