import rosbag
bag = rosbag.Bag('/home/erush91/data/radar/fog_garage_artifacts_0.bag')
for topic, msg, t in bag.read_messages(topics=['/os1_cloud_node/points']):
    print(msg)
bag.close()
