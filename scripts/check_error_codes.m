rosshutdown;
rosinit;

mes = rosmessage('moveit_msgs/GetPositionFKResponse');
val = mes.ErrorCode;

d = containers.Map;

props = properties(val);

for prop = 4:length(props)-1
    thisprop = props{prop};
    % d(key) = value
    value = thisprop;
    disp(value)
    key = string(val.(thisprop));
    disp(key)
    d(key) = value;
end


rosshutdown;