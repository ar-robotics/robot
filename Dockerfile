FROM arm64v8/ros:humble

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# create a non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir /home/${USERNAME}/.config && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config
