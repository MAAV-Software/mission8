#!/bin/sh

SSH_OPTS="-t"
SCRIPTS_TO_SYNC="software-run software-run-no-controller software-build software-killall"
PATHS_TO_SYNC="config/ ${SCRIPTS_TO_SYNC}"

for h in $(cat res/admin/hosts)
do
	ping -c 1 -W 2 ${h} > /dev/null

	if [ $? -eq 0 ]
	then
		ssh ${SSH_OPTS} maav@${h} "test -d /home/maav/software/config || mkdir /home/maav/software/config"

		for path in ${PATHS_TO_SYNC}
		do
			rsync -av --delete ${path} maav@${h}:/home/maav/software/${path}
		done
	fi
done
