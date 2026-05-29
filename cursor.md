maum@maum:~$ sudo journalctl -u ligo-mode-orchestrator.service -f
May 29 10:35:23 maum start.sh[8291]: [INFO] stopped: mode=odometry, map_name=map, exit_code=0, reason=service_shutdown, stop_timeout_sec=10.0, save_elapsed_sec=n/a
May 29 10:35:23 maum start.sh[8291]: [WARN] MQTT 연결 해제: reason=ReasonCode(Disconnect, 'Normal disconnection')
May 29 10:35:23 maum systemd[1]: ligo-mode-orchestrator.service: Deactivated successfully.
May 29 10:35:23 maum systemd[1]: Stopped LIGO MQTT Mode Orchestrator.
May 29 10:35:23 maum systemd[1]: ligo-mode-orchestrator.service: Consumed 11.550s CPU time.
May 29 10:35:23 maum systemd[1]: Started LIGO MQTT Mode Orchestrator.
May 29 10:35:24 maum start.sh[8718]: [INFO] MQTT 연결 시도 중: rms.bottle-tak.com:80
May 29 10:35:24 maum start.sh[8718]: [INFO] control_topic=80f7e77a602ea53d/control/mode, status_topic=80f7e77a602ea53d/control/mode_status
May 29 10:35:25 maum start.sh[8718]: [INFO] MQTT 연결됨: rms.bottle-tak.com:80
May 29 10:35:25 maum start.sh[8718]: [INFO] MQTT 구독 완료: 80f7e77a602ea53d/control/mode
May 29 10:35:45 maum start.sh[8942]: [INFO] [launch]: All log files can be found below /home/maum/.ros/log/2026-05-29-10-35-45-336819-maum-8942
May 29 10:35:45 maum start.sh[8942]: [INFO] [launch]: Default logging verbosity is set to INFO
May 29 10:35:45 maum start.sh[8942]: [INFO] [ligo_mapping-1]: process started with pid [8943]
May 29 10:35:45 maum start.sh[8942]: [INFO] [ligo_topic_to_mqtt.py-2]: process started with pid [8945]
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED is now ignored. Please set RCUTILS_LOGGING_USE_STDOUT and RCUTILS_LOGGING_BUFFERED_STREAM to control the stream and the buffering of log messages.
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] [WARN] [1780018545.525152843] [ligo]: Reset NMEAProcess
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] [INFO] [1780018545.525366047] [ligo]: [map] map_folder=/home/maum/last_navi/src/LIGO./PCD
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] mapping mode:0
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] nmea enable:0
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] indoor enable:0
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] odometry map-local mode:1
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] [INFO] [1780018545.526542317] [ligo]: [indoor/gicp] odometry map group: map_name=map dir=/home/maum/last_navi/src/LIGO./PCD/map
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] indoor.gicp_factor_sqrt_info_scale: 200
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] [INFO] [1780018545.526719064] [laserMapping]: lidar_type: 1
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] [WARN] [1780018545.526977184] [laserMapping]: [indoor/gicp] indoor.grid_map_dir=/home/maum/last_navi/src/LIGO./PCD/map invalid or empty — check * _grid2d.yaml
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] [WARN] [1780018545.527044850] [laserMapping]: [indoor/gicp] nmea_enable=false fallback: loading raw PCD directly: /home/maum/last_navi/src/LIGO./PCD/map/sub_map/sub_map.pcd
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] [WARN] [1780018545.614727865] [ligo]: ==== [indoor/gicp] REFERENCE MAP LOADED ==== path=/home/maum/last_navi/src/LIGO./PCD/map/sub_map/sub_map.pcd
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] [INFO] [1780018545.630670947] [laserMapping]: [indoor/gicp] nmea_enable=false: map-only odometry session started
May 29 10:35:45 maum start.sh[8942]: [ligo_mapping-1] ~~~~ debug file logging disabled
May 29 10:35:45 maum start.sh[8942]: [ligo_topic_to_mqtt.py-2] RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED is now ignored. Please set RCUTILS_LOGGING_USE_STDOUT and RCUTILS_LOGGING_BUFFERED_STREAM to control the stream and the buffering of log messages.
May 29 10:35:46 maum start.sh[8718]: [INFO] started: mode=odometry, map_name='', indoor_map_names=['map'], nmea_enable=False, pid=8942
May 29 10:35:46 maum start.sh[8942]: [ligo_topic_to_mqtt.py-2] [INFO] [1780018546.657392884] [ligo_topic_to_mqtt]: ROS->MQTT bridge started. mqtt=rms.bottle-tak.com:80 topics=80f7e77a602ea53d/position, 80f7e77a602ea53d/heading, 80f7e77a602ea53d/gps, 80f7e77a602ea53d/init_heading_icp, 80f7e77a602ea53d/ligo_mode
May 29 10:35:46 maum start.sh[8942]: [ligo_topic_to_mqtt.py-2] [INFO] [1780018546.903836206] [ligo_topic_to_mqtt]: MQTT connected
May 29 10:35:56 maum start.sh[8942]: [ligo_topic_to_mqtt.py-2] [INFO] [1780018556.633691373] [ligo_topic_to_mqtt]: MQTT health: connected=True, publish_fail_count=0, last_publish_ok_age_sec=9.726775407791138
May 29 10:36:06 maum start.sh[8942]: [ligo_topic_to_mqtt.py-2] [INFO] [1780018566.633579500] [ligo_topic_to_mqtt]: MQTT health: connected=True, publish_fail_count=0, last_publish_ok_age_sec=19.72671341896057
