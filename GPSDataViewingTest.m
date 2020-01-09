%% Initialize
close all
delete(instrfind({'Port'},{'COM8'}));
s = serial('COM8','BaudRate',115200);
fopen(s);

numOutputs = 12;     %This is the number of things we are outputing from arduino
data = [];          %Clear the data variables
current_y = [];                   
current_x = [];
alt = [];
delta_h = [];
error = [];
endCon = repmat(-1,numOutputs,1);  %Initialize the end transmission sequence
i = 1;

%% Set Up the GPS Plot Backround

close all;
hold on;
%These are the names of the data the arduino is outputting, in the correct
%order.
names = {'Pos Y:';'Pos X:';'Alt:';'Heading:';'Delta h:';'error:';'error_d:';'encder count:';'encdr target:';'Current Distance:';'P:';'I';'D:'};
%The image to put onto the graph, to help visualise the position data.
img = imread('uvic.jpg');

P1 = [164.270678003196      112.291244572924];      %Top Left Corner if the image, in meters
P2 = [ -173.929321996804	-195.008755427076];     %Bottom Right corner of the image, in meters

ycorner = [P1(2);P2(2)];
xcorner = [P2(1);P1(1)];

target_long       = 0;                              %Again, these are in meters, not degrees. I've set up P1 and P2 such that the target lat and long correspond to 0,0 of the picture
target_lat        = 0;                                  %I think this is easier than plotting the lat and long directly, as then you have to deal with scaling issues,
cntrl_radii       = 100;                                %and drawing the circles becomes much more annoying.
image('CData',img,'XData',xcorner,'YData',ycorner); %Add the image to the plot
axis equal;                                         %We want equal x and y axis 
circle(0,0,cntrl_radii,'w');                        %Draw the control circle.   

%% Recieve Data from Arduino
while(1)
   %This if statement will print the current index to the command window
   %every 10 iterations.
   if (mod(i,10) == 0)      %output the index every 10 iterations
      if i>1
          for j=0:log10(i-10)+1
              fprintf('\b'); % delete previous counter display
          end
      end
       fprintf('%d ', i);   
   end
   
   readData = fscanf(s,'%e');           %Read the incoming transmissionfrom the arduino
   if(isequal(readData,endCon))         %If it's the end transmission, break from the loop
      break;                             %I used this during simulated runs. It doesn't do anything with a live test.
   elseif (~isequal(size(readData),size(names)))       %If it's not the right size, it's probably gaabage
       disp('Invalid Data Read:');
       disp(readData);
   else
    data(:,i) = readData;           %Otherwise it's probably good. Write the data into the array
    i = i + 1;                       %iterate
    %Fill up all the variables with their corresponding row.
    current_y = data(1,:);                   
    current_x = data(2,:);
    alt = data(3,:);
    current_heading = data(4,:);
    delta_h = data(5,:);
    error = data(6,:);
    error_d = data(7,:);
    encoder_count = data(8,:);
    encoder_target = data(9,:);
    motor_dir = data(10,:);
    
   end
   figure(1)
   hold off
   image('CData',img,'XData',xcorner,'YData',ycorner)  %Draw the image.
   axis equal
   hold on
   plot(0, 0,'-wx')                 %Draw an x at the center
   circle(0,0,cntrl_radii,'w')      %Draw the control radius   
   plot(current_x,current_y,'gx')   %Plot the position vertices
   quiver(current_x,current_y,sind(current_heading),cosd(current_heading),1,'g');  %This plots the heading vector.
   set(1, 'Position',  [0, 0, 750, 500])        %This controls the figure window location.
   
   %Figure 2 shows all the data. 
   figure (2)
   clf;
   
   
   hold on;
   text(0.6,0.5,num2str(readData));
   text(0.2,0.5,names);
   hold off
   set(2, 'Position',  [750, 0, 400, 500])
   
end
%% Free the COM port

%This needs to be run at the end of each test, if matlab is to let go of
  %the arduino.
delete(instrfind({'Port'},{'COM8'}));