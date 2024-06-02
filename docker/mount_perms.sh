#!/bin/sh

USERNAME=${1:-johnnyappleseed}
UID=$(stat -c "%u" .)
GID=$(stat -c "%g" .)
SOCK_GID=$(stat -c "%g" /var/run/docker.sock)

groupadd --non-unique --gid $GID $USERNAME
groupadd --non-unique --gid $SOCK_GID docker_sock

useradd --home-dir ${HOME} \
	--gid $GID \
	--uid $UID \
	--password $(openssl passwd -1 $USERNAME) \
	--groups sudo,docker_sock \
	--shell /bin/bash \
	$USERNAME

touch ${HOME}/.sudo_as_admin_successful

su --login $USERNAME
