#!/usr/bin/bash
set -e

sed -i "s,__RMF_SERVER_URL__,${RMF_SERVER_URL},g" /opt/dashboard/index.html
sed -i "s,__TRAJECTORY_SERVER_URL__,${TRAJECTORY_SERVER_URL},g" /opt/dashboard/index.html
