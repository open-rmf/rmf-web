#!/bin/sh
while true
do
	echo "Sending logs to Fluentd"
  curl -X POST -d 'json={"foo":"bar"}' http://fluentd:9880/http-mock-myapp.log
	sleep 5
done