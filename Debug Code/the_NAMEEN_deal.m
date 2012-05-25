function [ Xbot, Ybot, Xorient, Yorient ] = the_NAMEEN_deal( x1, x2, x3, x4, y1, y2, y3, y4 )

% the_NAMEEN_deal takes the local x/y positions of four IR blobs and
% computes the global position of the robot with respect to the center of
% the hockey rink.
%   by Mike Kofron

clc

%% define constants
x_width_pixels = 1024;  % [pixels]
y_width_pixels = 768;
x_center_pixels = x_width_pixels/2;  % [pixels]
y_center_pixels = y_width_pixels/2;

X2 = 0;
X3 = 0;
Y2 = -14.5;
Y3 = 14.5;

%% shift input variables
% convert from [pixels] with corner (0,0) to [cm] with central (0,0)
x1 = -x1 + x_center_pixels;    % [cm]
x2 = -x2 + x_center_pixels;
x3 = -x3 + x_center_pixels;
x4 = -x4 + x_center_pixels;
y1 = y1 - y_center_pixels;
y2 = y2 - y_center_pixels;
y3 = y3 - y_center_pixels;
y4 = y4 - y_center_pixels;

%% compute known vectors
V_vert_vect = [X3-X2, Y3-Y2];
V_vert = norm(V_vert_vect);

%% compute unknown vectors
% the vectors that connect each IR blob with another, in local coordinates
v21_vect = [x2-x1, y2-y1]; % [cm]
v31_vect = [x3-x1, y3-y1];
v41_vect = [x4-x1, y4-y1];
v32_vect = [x3-x2, y3-y2];
v42_vect = [x4-x2, y4-y2];
v43_vect = [x4-x3, y4-y3];

v21 = norm(v21_vect);
v31 = norm(v31_vect);
v41 = norm(v41_vect);
v32 = norm(v32_vect);
v42 = norm(v42_vect);
v43 = norm(v43_vect);

%% compare local vectors with global vertical
% to find which points are points 2 and 3
vects_ = [ v21, v31, v41, v32, v42, v43 ];
[~,index_min] = min(vects_);
[~,index_max] = max(vects_);

if( index_max == 1 )
    v_vert = v21;
    x_vert1 = x1;   x_vert2 = x2;
    y_vert1 = y1;   y_vert2 = y2;
elseif( index_max == 2 )
    v_vert = v31;
    x_vert1 = x1;   x_vert2 = x3;
    y_vert1 = y1;   y_vert2 = y3;
elseif( index_max == 3 )
    v_vert = v41;
    x_vert1 = x1;   x_vert2 = x4;
    y_vert1 = y1;   y_vert2 = y4;
elseif( index_max == 4 )
    v_vert = v32;
    x_vert1 = x2;   x_vert2 = x3;
    y_vert1 = y2;   y_vert2 = y3;
elseif( index_max == 5 )
    v_vert = v42;
    x_vert1 = x2;   x_vert2 = x4;
    y_vert1 = y2;   y_vert2 = y4;
elseif( index_max == 6 )
    v_vert = v43;
    x_vert1 = x3;   x_vert2 = x4;
    y_vert1 = y3;   y_vert2 = y4;
else
    v_vert = 0;
    x_vert1 = 0;    x_vert2 = 0;
    y_vert1 = 0;    y_vert2 = 0;
end

%% compute scaling factors
scale = V_vert / v_vert ;  % [cm/pixel]

%% declare the local coordinates of the bot
% give the bot's position relative to rink center, in the coordinate system
% described by its own local x- and y- axes
xO = scale*( (x_vert1 + x_vert2)/2 );  % this is the local position of the origin
yO = scale*( (y_vert1 + y_vert2)/2 );
x1 = scale*x1;
x2 = scale*x2;
x3 = scale*x3;
x4 = scale*x4;
y1 = scale*y1;
y2 = scale*y2;
y3 = scale*y3;
y4 = scale*y4;
x_vert1 = scale*x_vert1;
x_vert2 = scale*x_vert2;
y_vert1 = scale*y_vert1;
y_vert2 = scale*y_vert2;

%% derive which blobs are actually 2 and 3
% use a variety of vectors to decide exactly which of your local blobs
% correspond to which global blobs
if( index_min == 1 )
    x_other1 = x1;  x_other2 = x2;
elseif( index_min == 2 )
    x_other1 = x1;  x_other2 = x3;
elseif( index_min == 3 )
    x_other1 = x1;  x_other2 = x4;
elseif( index_min == 4 )
    x_other1 = x2;  x_other2 = x3;
elseif( index_min == 5 )
    x_other1 = x2;  x_other2 = x4;
elseif( index_min == 6 )
    x_other1 = x3;  x_other2 = x4;
else
    x_other1 = 0;   x_other2 = 0;
end

if( x_vert1 == x_other1 )
    x_actually2 = x_vert2;
    x_actually3 = x_vert1;
    y_actually2 = y_vert2;
    y_actually3 = y_vert1;
elseif( x_vert2 == x_other1 )
    x_actually2 = x_vert1;
    x_actually3 = x_vert2;
    y_actually2 = y_vert1;
    y_actually3 = y_vert2;
elseif( x_vert1 == x_other2 )
    x_actually2 = x_vert2;
    x_actually3 = x_vert1;
    y_actually2 = y_vert2;
    y_actually3 = y_vert1;
elseif( x_vert2 == x_other2 )
    x_actually2 = x_vert1;
    x_actually3 = x_vert2;
    y_actually2 = y_vert1;
    y_actually3 = y_vert2;
else
    x_actually2 = 0;
    x_actually3 = 0;
    y_actually2 = 0;
    y_actually3 = 0;
end

%% find the angle between the local and global axes
% the angle, theta, is the angle between the y- and Y-axes, and it is
% calculated in order to transform local coordinates to global
theta = atan2( (x_actually3 - x_actually2), (y_actually3 - y_actually2) );
theta = 180*theta/pi;

alpha = -atan2(yO,xO);
alpha = 180*alpha/pi;
r = sqrt(xO^2 + yO^2);

phi = theta - alpha;

%% declare the global coordinates of the bot
% give the bot's position relative to rink center, in the coordinate system
% described by the global X- and Y- axes

Xbot = -r*cosd(phi);
Ybot = -r*sind(phi);

Xorient = cosd(theta);
Yorient = sind(theta);

%% debug
clc

x_actually2
x_actually3
y_actually2
y_actually3

end