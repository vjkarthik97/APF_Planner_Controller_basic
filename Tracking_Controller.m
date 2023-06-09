clc;
close all;
clear all;

load APF_data.mat
Reference = Robot_State;

N_samples = 120000;
del_T = 0.001;

Kp_v = 0.1;
Kp_omega = 0.1;

current_reference = Reference(1,:);
Unicycle_states = zeros(3,N_samples);
Unicycle_states(:,1) = [Start_x;Start_y;pi/2];

Robot_Position = Unicycle_states(1:2,k);
distance = Euclidean_distance(Robot_Position(1),Robot_Position(2),current_reference(1),current_reference(2));

ref_index = 1;


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

%% Controller Simulation
for k = 1:1:N_samples

    disp("In-Loop check")
    Robot_Position = Unicycle_states(1:2,k);
    
    current_reference = Reference(ref_index,:)';

    distance = Euclidean_distance(Robot_Position(1),Robot_Position(2),current_reference(1),current_reference(2));
    angle = Unicycle_states(3,k) - atan2(current_reference(2)-Robot_Position(2),+current_reference(1)-Robot_Position(1)+0.0001);

    if(distance<5)
        ref_index = ref_index + 1;
        if(ref_index>length(Reference))
            ref_index = length(Reference);
        end
    end

    v = Kp_v*distance;
    omega = -Kp_omega*angle;

    Unicycle_states(:,k+1) = Unicycle_states(:,k) + [cos(Unicycle_states(3,k)) 0;
                                                     sin(Unicycle_states(3,k)) 0;
                                                     0 1]*[v;omega]*del_T;
    

end

plot(Start_x,Start_y,'-o','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6])
hold on;
plot(Goal(1),Goal(2),'-o','MarkerSize',10,'MarkerEdgeColor','green','MarkerFaceColor',[.6 1 .6])
grid on;
plot(Unicycle_states(1,:),Unicycle_states(2,:),'LineWidth',2);
%an = animatedline(Robot_State(:,1),Robot_State(:,2));
%an = animatedline([0 0],[0 0],'Color','r','LineWidth',3);
%figure(2)

%h = plot(nan, nan, 'ko', 'MarkerSize', 7, 'MarkerFaceColor','b');   %yellow, filled, large
g = plot(nan, nan, 'ko', 'MarkerSize', 7, 'MarkerFaceColor','b');   %yellow, filled, large
for i = 1:100:length(Unicycle_states(1,:))
    
    %addpoints(an,Robot_State(i,1),Robot_State(i,2));
    %set(h, 'XData', Robot_State(1:i,1), 'YData', Robot_State(1:i,2));
    set(g, 'XData', Unicycle_states(1,1:1), 'YData', Unicycle_states(2,1:1));
    
    drawnow
end