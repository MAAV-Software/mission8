#!/bin/bash

source bash/functions.sh

enter_ansible

(cd res/admin && ansible-playbook bootstrap.yml)
