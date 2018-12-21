#!/bin/sh

SSH_OPTS="-t"
PATHS_TO_SYNC="CMakeLists.txt cmake/ doc/ include/ generated/ thirdparty/ src/ config/ drivers/"

for h in $(cat res/admin/hosts)
do
	ping -c 1 -W 2 ${h} > /dev/null

	if [ $? -eq 0 ]
	then
		ssh ${SSH_OPTS} maav@${h} "test -d /home/maav/software || mkdir /home/maav/software"

		for path in ${PATHS_TO_SYNC}
		do
			rsync -av --delete ${path} maav@${h}:/home/maav/software/${path}
		done

		ssh ${SSH_OPTS} maav@${h} /usr/local/bin/nav_build_install
	fi
done
