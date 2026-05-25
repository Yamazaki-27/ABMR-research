#!/bin/bash

set -e

CONTAINER_NAME="atmobi"
LOCAL_FILE="steer_auto.py"
CONTAINER_FILE="/home/steer_auto.py"

echo "PythonファイルをDockerコンテナへコピーします..."
docker cp "$LOCAL_FILE" "$CONTAINER_NAME:$CONTAINER_FILE"

echo "Dockerコンテナ内でROS環境を読み込んでPythonコードを起動します..."
docker exec -it "$CONTAINER_NAME" /bin/bash -lc "cd /home && source /opt/ros/melodic/setup.bash && python steer_auto.py"
