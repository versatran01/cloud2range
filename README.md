# cloud2range

Convert point cloud to range image and back. This allows point cloud to be saved
as range image, which drastically reduces the required storage, by at least 10x.

Only tested with VLP16.

Assumes `cut_angle=0`, meaning the velodyne driver publishes full 360 point cloud.
