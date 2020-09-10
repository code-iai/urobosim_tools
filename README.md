# urobosim_tools

Example call to send a transformed pose of base_footprint in map to a special topic:
````
python frame_in_other_frame_publisher.py --src /base_footprint --target /map --topic /tf_replay
````
