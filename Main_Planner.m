clc;
close all;
clear all;

%[Start,Goal,step_size,ObsList,ObsNum] = DataRetrievelText();

syms x y Att_Potential_close Att_Potential_far Rep_Potential_overall
%syms Rep_Potential [1 ObsNum]


%% Initializations
Start_x = 35;
Start_y = 10;

Final_Goal_x = 88;
Final_Goal_y = 20;

Goal = [Final_Goal_x Final_Goal_y];

Robot_State(1,1) = Start_x;
Robot_State(1,2) = Start_y;

k = 1;

chi = 0.8;
d_star = 2;
eta = 0.8;
Q_star = 8;

step_size = 0.5;


Att_Potential_close = 0.5*chi*(Euclidean_distance(x,y,Goal(1),Goal(2)))^2;
Att_Potential_far = d_star*chi*Euclidean_distance(x,y,Goal(1),Goal(2)) - 0.5*chi*(d_star)^2;

grad_Att_close = gradient(Att_Potential_close,[x y]);
grad_Att_far = gradient(Att_Potential_far,[x y]);
%grad_Rep = gradient(Rep_Potential,[x y]);

Robot_x = Start_x;
Robot_y = Start_y;

lidar = rangeSensor;
lidar.Range = [0,100];

%% Reading Image
image = imread('Custom_Map_Maze_criss_cross.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
refMap = binaryOccupancyMap(bwimage);
%show(grid)

%refMap = binaryOccupancyMap(simpleMap,1);
refFigure = figure('Name','SimpleMap');
show(refMap);
hold on;
plot(Start_x,Start_y,'^r','MarkerFaceColor','#FF0000','MarkerSize',10)
hold on;
plot(Final_Goal_x,Final_Goal_y,'sg','MarkerFaceColor','#00FF00','MarkerSize',10)
hold on;

%mapdimx = 100;
%mapdimy = 100;

%map = binaryOccupancyMap(mapdimy,mapdimx,10);
%mapFigure = figure('Name','Unknown Map');
%show(map);

while(Euclidean_distance(Robot_State(k,1),Robot_State(k,2),Goal(1),Goal(2))>2)
    %% Obtaining the range information
    Robot_x = Robot_State(k,1);
    Robot_y = Robot_State(k,2);

    position = [Robot_x Robot_y 0];
    %Robot_x = position(1);
    %Robot_y = position(2);
    [ranges, angles] = lidar(position,refMap);
    scan = lidarScan(ranges,angles);
    validScan = removeInvalidData(scan,'RangeLimits',[0,lidar.Range(2)]);

    syms x y
    syms Rep_Potential [1 length(ranges)]

    
    for i = 1:1:length(ranges)
        
        closest_distance = Euclidean_distance(x,y,Robot_x + ranges(i)*cos(angles(i)),Robot_y + ranges(i)*sin(angles(i)));

        if(ranges(i)<=Q_star)
            Rep_Potential(i) = 0.5*eta*((1/closest_distance) - (1/Q_star))^2; 
        else
            Rep_Potential(i) = 0;
            %continue;
        end
    %end
    end
    Rep_Potential_overall = Rep_Potential(1);

    if(length(ranges)>1)
        
        for j = 2:1:length(ranges)
            Rep_Potential_overall = Rep_Potential_overall + Rep_Potential(j);
        end
    end
    
    grad_Rep = gradient(Rep_Potential_overall,[x y]);

    x = Robot_State(k,1);
    y = Robot_State(k,2);

    if(Euclidean_distance(Robot_State(k,1),Robot_State(k,2),Goal(1),Goal(2))<d_star)
        v = -double(subs(grad_Att_close) + subs(grad_Rep)); %Using near attraction potential
    else
        v = -double(subs(grad_Att_far) + subs(grad_Rep)); %Using far attraction potential
    end

    Robot_State(k+1,:) = Robot_State(k,:) + transpose(v)*step_size/norm(v); 
    k = k+1;
    
end

%% Plotting

%figure(1)
%plot(Robot_State(:,1),Robot_State(:,2),'LineWidth',2)
%hold on;

plot(Start_x,Start_y,'-o','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6])
hold on;
plot(Goal(1),Goal(2),'-o','MarkerSize',10,'MarkerEdgeColor','green','MarkerFaceColor',[.6 1 .6])
grid on;
%an = animatedline(Robot_State(:,1),Robot_State(:,2));
%an = animatedline([0 0],[0 0],'Color','r','LineWidth',3);
%figure(2)

h = plot(nan, nan, 'ko', 'MarkerSize', 7, 'MarkerFaceColor','b');   %yellow, filled, large
for i = 1:1:length(Robot_State(:,1))
    
    %addpoints(an,Robot_State(i,1),Robot_State(i,2));
    set(h, 'XData', Robot_State(1:i,1), 'YData', Robot_State(1:i,2));
    
    drawnow
end
%comet(Robot_State(:,1),Robot_State(:,2))

%% Writing to an output text file

%fileID = fopen('output.txt','w');
%formatSpec = '%f,%f\n';
%for i = 1:1:length(Robot_State(:,1))
    %fprintf(fileID,formatSpec,Robot_State(i,1),Robot_State(i,2));
%end
