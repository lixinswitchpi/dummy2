% A demo about to use matlab ros2 to talk to rboot
clear;
clear;
node_loop = ros2node('topic_rboot_loop_pub');
pause(1);
node_pos = ros2node('topic_rboot_pos_pub');
pause(1);
% first put all acuators to loop mode
loopPub = ros2publisher(node_loop,"/loop","std_msgs/UInt8");
msg_loop = ros2message(loopPub);
msg_loop.data = uint8(0);
class(msg_loop.data)
send(loopPub, msg_loop);
pause(1);
msg_loop.data = uint8(1);
class(msg_loop.data)
send(loopPub, msg_loop);
pause(1);
% then we send out node's position
posPub = ros2publisher(node_pos,"/position","rboot_interface/RbootPosition");
msg_pos = ros2message(posPub);

% how many acuators
c = 0;
for i = [60.0 60.0 -60.0 60 60 60]
    disp(i);
    c = c + 1;
    msg_pos.nodeid = int32(c);
    disp(c);
    msg_pos.position = single(i);
    send(posPub, msg_pos);
end
pause(3);
c = 0;
for i = [0 0 0 0 0 0]
    disp(i);
    c = c + 1;
    msg_pos.nodeid = int32(c);
    disp(c);
    msg_pos.position = single(i);
    send(posPub, msg_pos);
end
pause(3);
% finally, put all acuators to idle mode
msg_loop.data = uint8(0);
send(loopPub, msg_loop);

% msg_pos.nodeid = int32(1);
% msg_pos.position = single(60.0);
% class(msg_pos)
% send(posPub, msg_pos);
% pause(3);
% msg_pos.position = single(0.0);
% send(posPub, msg_pos);



