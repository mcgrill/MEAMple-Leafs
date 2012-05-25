%%Wii sensor visualization
% Reads in data from microcontroller and displays visual location of points.
% Uncomment rawStarData to keep the IR points in logs.  You can then test
% these with the log testing script.
% by Nick McGill, based off of code by Jonathan Fiene

%%Initialize program and USB port
clear all
close all
%M1USB = serial('COM4','Baudrate', 9600); %where port is either 'COM#' in Windows or '/dev/tty.usbmodem#' in OS X.
M1USB = serial('/dev/tty.usbmodem411','Baudrate',9600);
fopen(M1USB);
flushinput(M1USB);
scrsz = get(0,'ScreenSize');

%% Set recording length
maxTime = 10; %Seconds
maxPoints = 20; % max number of data points displayed at a time
xpos = zeros(maxPoints,1);%1-100
ypos = zeros(maxPoints,1);%1-100
x = 1:1:maxPoints;
rawStarData = zeros(1,8);

%% Set up figure
figure(1);%'Position',[1 1 scrsz(3) scrsz(4)])

subplot(2,1,1);
title('Wii Blob Plotting');
axis equal
axis([0 1024 0 768]);
grid on
hold on

%% Initialize plot data to zero 
position1X = 0;
position1Y = 0;
size1 = 0;

position2X = 0;
position2Y = 0;
size2 = 0;

position3X = 0;
position3Y = 0;
size3 = 0;

position4X = 0;
position4Y = 0;
size4 = 0;

% initialize plot handles
point1 = plot(position1X,position1Y,'ro','MarkerSize',5,'MarkerFaceColor',[.7 0 0]);
point2 = plot(position2X,position2Y,'go','MarkerSize',5,'MarkerFaceColor',[0 .7 0]);
point3 = plot(position3X,position3Y,'bo','MarkerSize',5,'MarkerFaceColor',[0 0 .7]);
point4 = plot(position4X,position4Y,'ko','MarkerSize',5,'MarkerFaceColor',[0 0 0]);

%% Set up figure subplot2
subplot(2,1,2);
title('Global Bot Position');
axis([-150 150 -80 80]);
grid on
hold on

Xbot = 0;
Ybot = 0;
Xorient = 0;
Yorient = 0;
m2_Xorient = 0;
m2_Yorient = 0;

% initialize plot handles
rect1 = rectangle('Position',[-120,-60,240,120], 'Curvature', [.25,.25], 'EdgeColor', 'Blue', 'LineWidth', 2);
bot_pos = plot(Xbot,Ybot,'ko','MarkerSize',5,'MarkerFaceColor',[0 0 0]);
bot_orient = plot([0,Xorient],[0,Yorient],'b');


%% Send initial packet to get first set of data from microcontroller
fwrite(M1USB,1);
time = 0;
i=0;
tic

%Run program for specified amount of time
try
while 1
    %Read in the data and send a confirmation packet
    a2 = fgetl(M1USB);
    fwrite(M1USB,1); 

    %Parse the data sent by the microcontroller
    position1X=hex2dec(a2(1:4));
    position1Y=hex2dec(a2(5:8));
    size1=hex2dec(a2(9:12));

    if((position1X == 1023) && (position1Y == 1023))
        %Lost sight of the point - make plotted size very small
        size1 = -4.99;
    end

    position2X = hex2dec(a2(13:16));
    position2Y = hex2dec(a2(17:20));
    size2 = hex2dec(a2(21:24));

    if((position2X == 1023) && (position2Y == 1023))
        %Lost sight of the point - make plotted size very small
        size2 = -4.99;
    end

    position3X = hex2dec(a2(25:28));
    position3Y = hex2dec(a2(29:32));
    size3 = hex2dec(a2(33:36));

    if((position3X == 1023) && (position3Y == 1023))
        %Lost sight of the point - make plotted size very small
        size3 = -4.99;
    end

    position4X = hex2dec(a2(37:40));
    position4Y = hex2dec(a2(41:44));
    size4 = hex2dec(a2(45:48));

    if((position4X == 1023) && (position4Y == 1023))
        %Lost sight of the point - make plotted size very small
        size4 = -4.99;
    end

    %Use data obtained from microcontroller to set position and size of points
    set(point1,'XData',position1X)
    set(point1,'YData',position1Y)
    set(point1,'MarkerSize',5+size1)

    set(point2,'XData',position2X)
    set(point2,'YData',position2Y)
    set(point2,'MarkerSize',5+size2)

    set(point3,'XData',position3X)
    set(point3,'YData',position3Y)
    set(point3,'MarkerSize',5+size3)

    set(point4,'XData',position4X)
    set(point4,'YData',position4Y)
    set(point4,'MarkerSize',5+size4)
    
    [ Xbot, Ybot, Xorient, Yorient ] = the_NAMEEN_deal(...
        position1X, position2X, position3X, position4X,...
        position1Y, position2Y, position3Y, position4Y);
    

    set(bot_pos,'XData',Xbot)
    set(bot_pos,'YData',Ybot)
    set(bot_pos,'MarkerSize',5+size4)
    
    set(bot_orient,'XData',Xbot+10*[0,Xorient])
    set(bot_orient,'YData',Ybot+10*[0,Yorient])
    
    Xbot;
    Ybot;
    
    
    
    % retieve and store data
%     xpos = circshift(xpos,-1);%film...
%     xpos(maxPoints,:) = Xbot;
%     ypos = circshift(ypos,-1);%film...
%     ypos(maxPoints,:) = Ybot;
    
%     hold on
%     clf; rectangle('Position',[-120,-60,240,120], 'Curvature', [.5,.5], 'EdgeColor', 'Red', 'LineWidth', 2);
%     hold on
%     plot(Xbot, Ybot, 'ro');
%     axis([-120 120 -60 60]);
    
%     figure(3);
%     hold on
%     grid on
%     plot(x,xpos(:,1),':or');
%     plot(x,ypos(:,1),':ok');
%     pause(0.5);
    
    drawnow
    i=i+1;
    
%     rawStarData(i,1) = position1X;
%     rawStarData(i,2) = position2X;
%     rawStarData(i,3) = position3X;
%     rawStarData(i,4) = position4X;
%     rawStarData(i,5) = position1Y;
%     rawStarData(i,6) = position2Y;
%     rawStarData(i,7) = position3Y;
%     rawStarData(i,8) = position4Y;
%     rawStarData(i,9) = size1;
%     rawStarData(i,10) = size2;
%     rawStarData(i,11) = size3;
%     rawStarData(i,12) = size4;
    
    
    
    time = toc;
end 
catch
    %Close serial object
    fclose(M1USB); 
end