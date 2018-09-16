#!/bin/sh

for h in $(cat res/admin/hosts)
do
	ssh-copy-id maav@${h}
done
