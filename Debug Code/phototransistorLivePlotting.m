%%	 bot ir readings
% reads all of the ir sensor data and plots it realtime
% by Nick McGill

%% Initialize program and USB port

% close any existing open port connections
if(exist('M1USB'))
    fclose(M1USB);
else
    fclose(instrfindall);
end

clear all
close all
M1USB = serial('COM7','Baudrate', 9600); %where port is either 'COM#' in Windows or '/dev/tty.usbmodem#' in OS X.
%M1USB = serial('/dev/tty.usbmodem411','Baudrate',9600);
disp('poop')
fopen(M1USB);
flushinput(M1USB);

%% Send initial packet to get first set of data from microcontroller
fwrite(M1USB,1);
time = 0;
i=0;
tic

maxPoints = 20; % max number of data points displayed at a time
ir1pos = zeros(maxPoints,1);%1-100
ir2pos = zeros(maxPoints,1);%1-100
ir3pos = zeros(maxPoints,1);%1-100
ir4pos = zeros(maxPoints,1);%1-100
t = 1:1:maxPoints;


irData = zeros(1,4);

%Run program for specified amount of time
try
while 1
    %Read in the data and send a confirmation packet
    a2 = fgetl(M1USB);
    fwrite(M1USB,1); 

    %Parse the data sent by the microcontroller
	% expecting data in the form: [uint IR1, uint IR2, uint IR3, uint IR4]
    IR1 = hex2dec(a2(1:4));
    IR2 = hex2dec(a2(5:8));
    IR3 = hex2dec(a2(9:12));
	IR4 = hex2dec(a2(13:16));

% retieve and store data
    ir1pos = circshift(ir1pos,-1);%film...
    ir1pos(maxPoints,:) = IR1;
	ir2pos = circshift(ir2pos,-1);%film...
    ir2pos(maxPoints,:) = IR2;
	ir3pos = circshift(ir3pos,-1);%film...
    ir3pos(maxPoints,:) = IR3;
	ir4pos = circshift(ir4pos,-1);%film...
    ir4pos(maxPoints,:) = IR4;
    
    i=i+1;
    
    disp('sensing');
    disp(i);
    
    irData(i,1) = IR1;
    irData(i,2) = IR2;
    irData(i,3) = IR3;
    irData(i,4) = IR4;
    
    
    figure(1);
    clf
    hold on
    grid on
    
    
    subplot(2,2,1);
    plot(t,ir1pos(:,1),':or');
    title('front left ir sensor (f0)');
    axis([0 maxPoints 0 1028]);
    
    subplot(2,2,2);
    plot(t,ir2pos(:,1),':or');
    title('front right ir sensor (f1)');
    axis([0 maxPoints 0 1028]);
    
    subplot(2,2,3);
    plot(t,ir3pos(:,1),':or');
    title('back right ir sensor (f4)');
    axis([0 maxPoints 0 1028]);
    
    subplot(2,2,4);
    plot(t,ir4pos(:,1),':or');
    title('back left ir sensor (f5)');
    axis([0 maxPoints 0 1028]);
    
    pause(.01);
    hold off
    
    
    time = toc;
end 
catch
    %Close serial object
    fclose(M1USB);
end