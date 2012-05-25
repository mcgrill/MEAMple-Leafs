%%Wii sensor visualization
%loads the raw wii sensor data and runs it through a matlab script to
%obtain position.  this calculated location is plotted on a graph of the rink.
% by Nick McGill

%%Initialize program and USB port
clear all
close all
scrsz = get(0,'ScreenSize');
load ../IRBlobLogs/leftside_xstartstowardorigin_rotateclockwise.mat
%load ../IRBlobLogs/leftside_xtowardorigin_bottomtotop.mat
%load ../IRBlobLogs/bigFigure8.mat
%load ../IRBlobLogs/plusSign.mat
%load ../IRBlobLogs/random.mat
%load ../IRBlobLogs/smallFigure8.mat




%% Set up figure 1
figure(1);
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

i=1;

while( i < length(rawStarData) )
    
    pause(.01);
    
    [ Xbot, Ybot, Xorient, Yorient ] = the_NAMEEN_deal(...
        rawStarData(i,1), rawStarData(i,2), rawStarData(i,3), rawStarData(i,4),...
        rawStarData(i,5), rawStarData(i,6), rawStarData(i,7), rawStarData(i,8) );


    set(bot_pos,'XData',Xbot);
    set(bot_pos,'YData',Ybot);
    set(bot_pos,'MarkerSize',5);

    set(bot_orient,'XData',Xbot+10*[0,Xorient]);
    set(bot_orient,'YData',Ybot+10*[0,Yorient]);
    
    i = i+1;
    
end