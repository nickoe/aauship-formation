% This file is supposed to replay measurement data from a real seatrail as
% it is presented to ROS. So this should act as a fake
% sensor-decode-node.py (publishes IMU through the /samples topic) and the
% lli-node.py (publishes GPS data thorugh the /gps1 topic).