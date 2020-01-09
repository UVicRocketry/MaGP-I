%% Initialize
close all
delete(instrfind({'Port'},{'COM8'}));       %Update the COM and baudrate with whatever's in the current setup
s = serial('COM8','BaudRate',1000000);
fopen(s);
numOutputs = 6;     %This is the number of things we are outputing from arduino
data = [];          %Clear the data variable
i = 1;

endCon = repmat(-1,numOutputs,1);  %Initialize the end transmission sequence



%% Recieve Data from Arduino
while(1)
   if (mod(i,10) == 0)      %output the index every 10 iterations
      if i>1
          for j=0:log10(i-10)+1
              fprintf('\b'); % delete previous counter display
          end
      end
       fprintf('%d ', i);   
   end
   
   readData = fscanf(s,'%e');           %read the incoming transmissionfrom the arduino
   if(isequal(readData,endCon))         %If it's the end transmission, break from the loop
      break
   elseif (~isequal(size(readData),size(endCon)))       %If it's not the right size, it's probably gabage
       disp('Oh NO!');
       disp(i);
   else
    data(:,i) = readData;           %Otherwise it's probably good. Write the data into the array
    i = i + 1;                      %iterate
   end
end

disp(' ');
disp('End Reached');

%% Make Pretty Graphs
close all
dist = 0;
disth = 0;
lat = data(1,:);
long = data(2,:);
alt = data(3,:);


figure(1)

plot3(long,lat,alt)
xlabel('long');
ylabel('lat');
zlabel('alt');
axis vis3d
daspect([1 1 1])

target_lat   = 0; 
target_long  = 0;
cntrl_radii       = [1000, 700, 400, 200];   
cntrl_radii_error   = [100, 70, 40, 20];  
colors = ['b','g','c','r'];
i=0;

figure(2)
hold on;
plot(target_long, target_lat,'-x')
for i=1 : 1: 4
    circle(target_long,target_lat,cntrl_radii(i),colors(i));
end

axis equal;
plot(long,lat,'k')
xlabel('long');
ylabel('lat');
hold off

for i=2:1:numel(lat)
    dist = dist + sqrt((lat(i-1)-lat(i))^2+(long(i-1)-long(i-1))^2+(alt(i)-alt(i-1))^2);
    disth = disth + sqrt((lat(i-1)-lat(i))^2+(long(i-1)-long(i-1))^2);
end
fprintf('The total distance traveled is:  %6.2fm.\n',dist);
fprintf('The horizontal distance traveled is:  %6.2fm.\n',disth);

%data = [];



%% Do some GPS Stuff
long1 = -123;
long2 = -123.5;

lat1 = 49;
lat2 = 49.5;

h = 36485.4335;
w = 55613.3359;

longreal = (long*(long2-long1)+long1*h)/h;
latreal = (lat*(lat2-lat1)+lat1*w)/w;

geoshow(latreal,longreal)

%% End Stuff
delete(s);
delete(instrfind({'Port'},{'COM8'}));