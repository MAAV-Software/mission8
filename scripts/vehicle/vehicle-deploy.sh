#!/bin/sh

SSH_OPTS="-t"
PATHS_TO_SYNC="CMakeLists.txt cmake/ doc/ lib/ src/ atomcore/ config/ april/"

for h in $(cat res/admin/hosts)
do
	ping -c 1 -W 2 ${h} > /dev/null

	if [ $? -eq 0 ]
	then
		ssh ${SSH_OPTS} maav@${h} "test -d /home/maav/nav || mkdir /home/maav/nav"

		for path in ${PATHS_TO_SYNC}
		do
			rsync -av --delete ${path} maav@${h}:/home/maav/nav/${path}
		done

		ssh ${SSH_OPTS} maav@${h} /usr/local/bin/nav_build_install
	fi
done
