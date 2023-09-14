%{
    Programmer: Sai Narayan
    Assessment: CMPEN 454 - Project 2
    Date:       22 July 2023
    Professor:  Dr. Mohamed Almekkawy
%}

clc, clear, close all

%% Profile on - Analyzing the efficiency of implementation

profile on

%% Loading the input data
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';

vue2video = VideoReader(filenamevue2mp4);
vue4video = VideoReader(filenamevue4mp4);

load('Subject4-Session3-Take4_mocapJoints.mat');
load('vue2CalibInfo.mat');
load('vue4CalibInfo.mat');

numFrames=size(mocapJoints,1);

%% Initialize arrays to store error for each joint

j1 = zeros(1,numFrames);
j2 = zeros(1,numFrames);
j3 = zeros(1,numFrames);
j4 = zeros(1,numFrames);
j5 = zeros(1,numFrames);
j6 = zeros(1,numFrames);
j7 = zeros(1,numFrames);
j8 = zeros(1,numFrames);
j9 = zeros(1,numFrames);
j10 = zeros(1,numFrames);
j11 = zeros(1,numFrames);
j12 = zeros(1,numFrames);

avgError = zeros(1,numFrames); 


%% For all frames in the video
for mocapFnum = 1:numFrames
    
    %2.1 Reading the 3D joint data
    x = mocapJoints(mocapFnum,:,1); %array of 12 X coordinates
    y = mocapJoints(mocapFnum,:,2); % Y coordinates
    z = mocapJoints(mocapFnum,:,3); % Z coordinates
    conf = mocapJoints(mocapFnum,:,4); %confidence values
    
    if conf==1
        
        % 2.2 Reading camera parameters
        vue2Pmat = vue2.Pmat(:,4);
        vue4Pmat = vue4.Pmat(:,4);
        vue2Loc = -(vue2.Rmat.')*vue2Pmat;
        vue4Loc = -(vue4.Rmat.')*vue4Pmat;
        
        %2.3 Projecting 3D points into 2D pixel locations
        vue2Proj = zeros(3,12);
        vue4Proj = zeros(3,12);
        
        % projection for each joint
            for j = 1:12
                vue2Proj(:,j) = project3Dto2D(x(j),y(j),z(j), vue2);     
                vue4Proj(:,j) = project3Dto2D(x(j),y(j),z(j), vue4);    
            end
        
        % 2.4 Triangulation back into a set of 3D scene points
        
        % We employ the equation Ax=b
        b = vue4Loc-vue2Loc;
        
        % For each joint
        p = zeros(3,12);
           for j = 1:12
                vue2Div = vue2.Kmat\vue2Proj(:,j);     
                vue4Div = vue4.Kmat\vue4Proj(:,j);
                
                % Viewing vue2 & vue4
                vue2View = (vue2.Rmat.')* vue2Div;  
                vue4View = (vue4.Rmat.') * vue4Div; 
                crossDiv = cross(vue2View,vue4View)/abs(cross(vue2View,vue4View));
                A(:,1) = vue2View;       
                A(:,2) = -vue4View;      
                A(:,3) = crossDiv(:,3);    
               
                % inv is a(-1)* b
                inv = A\(b);      
                p1 = vue2Loc + (inv(1)*vue2View); 
                p2 = vue4Loc + (inv(2)*vue4View);
                
                % world coordinate matrix
                p(:,j) = (p1 + p2)/2;    
           end
        
        % 2.5 Measure error between triangulated and original 3D point
        % Defining the matrix for errors
        avg1 = 0;
        mainError = zeros(1,12);  
        for j = 1:12 % for each joint
            % calculate euclidean distance
            mainError(j) = sqrt((x(j) - p(1,j)).^2 + (y(j) - p(2,j)).^2 + (z(j) - p(3,j)).^2);   
            avg1 = mainError(j) + avg1;
        end
        %Error for point to pixel projection and going back to point (3D) per frame
        
        j1(mocapFnum) = mainError(1); 
        j2(mocapFnum) = mainError(2);
        j3(mocapFnum) = mainError(3);
        j4(mocapFnum) = mainError(4);
        j5(mocapFnum) = mainError(5);
        j6(mocapFnum) = mainError(6);
        j7(mocapFnum) = mainError(7);
        j8(mocapFnum) = mainError(8);
        j9(mocapFnum) = mainError(9);
        j10(mocapFnum) = mainError(10);
        j11(mocapFnum) = mainError(11);
        j12(mocapFnum) = mainError(12);
        avgError(mocapFnum) = avg1/12;
    end
end

%% Calculate Statistics for each joint

% Mean
meanMat = [mean(j1(:)),mean(j2(:)),mean(j3(:)),mean(j4(:)),mean(j5(:)),mean(j6(:)),mean(j7(:)),mean(j8(:)),mean(j9(:)),mean(j10(:)),mean(j11(:)),mean(j12(:))];
disp("Mean error of Joints 1-12");
disp(meanMat);
disp("Overall Mean");
disp(mean(meanMat));

% Standard Deviation
stdMat = [std(j1(:)),std(j2(:)),std(j3(:)),std(j4(:)),std(j5(:)),std(j6(:)),std(j7(:)),std(j8(:)),std(j9(:)),std(j10(:)),std(j11(:)),std(j12(:))];
disp("Standard deviation of error of Joints 1-12");
disp(stdMat);
disp("Overall Standard Deviation");
disp(mean(stdMat));

% Minimum 
minMat = [min(j1(:)),min(j2(:)),min(j3(:)),min(j4(:)),min(j5(:)),min(j6(:)),min(j7(:)),min(j8(:)),min(j9(:)),min(j10(:)),min(j11(:)),min(j12(:))];
disp("Minimum error of Joints 1-12");
disp(minMat);
disp("Overall Minimum");
disp(mean(minMat));

% Median
medianMat = [median(j1(:)),median(j2(:)),median(j3(:)),median(j4(:)),median(j5(:)),median(j6(:)),median(j7(:)),median(j8(:)),median(j9(:)),median(j10(:)),median(j11(:)),median(j12(:))];
disp("Median error of Joints 1-12");
disp(medianMat);
disp("Overall Median");
disp(mean(medianMat));

% Maximum
maxMat = [max(j1(:)),max(j2(:)),max(j3(:)),max(j4(:)),max(j5(:)),max(j6(:)),max(j7(:)),max(j8(:)),max(j9(:)),max(j10(:)),max(j11(:)),max(j12(:))];
disp("Maximum error of Joints 1-12");
disp(maxMat);
disp("Overall Maximum");
disp(mean(maxMat));

%% Plotting

% Plot 1: error graphs for joints
figure(1)
j = 1:numFrames;
plot(j, avgError, 'g.', 'MarkerSize',0.33, 'LineWidth', 0.33);
axis([0 27000 0 10.^(-11.5)]);
ylabel('Average Error');
xlabel('mocapFnum');
title('CORRELATION between Average Error & mocapFnum');

% 2.1 & 2.2 Outputing points on image according to frame number
mocapFnum = 11111;
x = mocapJoints(mocapFnum,:,1);      % array of 12 X coordinates
y = mocapJoints(mocapFnum,:,2);      % Y coordinates
z = mocapJoints(mocapFnum,:,3);      % Z coordinates
conf = mocapJoints(mocapFnum,:,4);   % confidence values
vue2Proj = zeros(3,12);
vue4Proj = zeros(3,12);
for j = 1:12
    vue2Proj(:,j) = project3Dto2D(x(j),y(j),z(j), vue2); 
    vue4Proj(:,j) = project3Dto2D(x(j),y(j),z(j), vue4);
end

% Plot 2: Vue2 frame plot

figure(2)
%the (50/100) factor is here to account for the difference in frame 
%rates between video (50 fps) and mocap (100 fps).
vue2video.CurrentTime = (mocapFnum-1)*(50/100)/vue2video.FrameRate;
vid2Frame = readFrame(vue2video);
imshow(vid2Frame);
title(['Vue2 Frame Plot at mocapFnum = ', num2str(mocapFnum)]);

hold on
for j = 1:12
    plot(vue2Proj(1,j),vue2Proj(2,j), 'g.', 'MarkerSize',12, 'LineWidth', 10);  
end
% To connect the points together 
plot([vue2Proj(1,1) vue2Proj(1,4)], [vue2Proj(2,1) vue2Proj(2,4)], 'r');
plot([vue2Proj(1,1) vue2Proj(1,2)], [vue2Proj(2,1) vue2Proj(2,2)], 'r');
plot([vue2Proj(1,2) vue2Proj(1,3)], [vue2Proj(2,2) vue2Proj(2,3)], 'r');
plot([vue2Proj(1,4) vue2Proj(1,5)], [vue2Proj(2,4) vue2Proj(2,5)], 'r');
plot([vue2Proj(1,5) vue2Proj(1,6)], [vue2Proj(2,5) vue2Proj(2,6)], 'r');
plot([vue2Proj(1,7) vue2Proj(1,10)], [vue2Proj(2,7) vue2Proj(2,10)], 'r');
plot([vue2Proj(1,7) vue2Proj(1,8)], [vue2Proj(2,7) vue2Proj(2,8)], 'r');
plot([vue2Proj(1,8) vue2Proj(1,9)], [vue2Proj(2,8) vue2Proj(2,9)], 'r');
plot([vue2Proj(1,10) vue2Proj(1,11)], [vue2Proj(2,10) vue2Proj(2,11)], 'r');
plot([vue2Proj(1,11) vue2Proj(1,12)], [vue2Proj(2,11) vue2Proj(2,12)], 'r');
plot([vue2Proj(1,1) vue2Proj(1,7)], [vue2Proj(2,1) vue2Proj(2,7)], 'r');
plot([vue2Proj(1,4) vue2Proj(1,10)], [vue2Proj(2,4) vue2Proj(2,10)], 'r');

% Plot 3: Vue4 frame plot

figure(3)
%the (50/100) factor is here to account for the difference in frame 
%rates between video (50 fps) and mocap (100 fps).
vue4video.CurrentTime = (mocapFnum-1)*(50/100)/vue4video.FrameRate;
vid4Frame = readFrame(vue4video);
imshow(vid4Frame);
title(['Vue4 Frame Plot at mocapFnum = ', num2str(mocapFnum)]);

hold on
for j = 1:12
    plot(vue4Proj(1,j), vue4Proj(2,j), 'g.', 'MarkerSize',12, 'LineWidth', 10);
end 
% To connect the points together 
plot([vue4Proj(1,1) vue4Proj(1,4)], [vue4Proj(2,1) vue4Proj(2,4)], 'b');
plot([vue4Proj(1,1) vue4Proj(1,2)], [vue4Proj(2,1) vue4Proj(2,2)], 'b');
plot([vue4Proj(1,2) vue4Proj(1,3)], [vue4Proj(2,2) vue4Proj(2,3)], 'b');
plot([vue4Proj(1,4) vue4Proj(1,5)], [vue4Proj(2,4) vue4Proj(2,5)], 'b');
plot([vue4Proj(1,5) vue4Proj(1,6)], [vue4Proj(2,5) vue4Proj(2,6)], 'b');
plot([vue4Proj(1,7) vue4Proj(1,10)], [vue4Proj(2,7) vue4Proj(2,10)], 'b');
plot([vue4Proj(1,7) vue4Proj(1,8)], [vue4Proj(2,7) vue4Proj(2,8)], 'b');
plot([vue4Proj(1,8) vue4Proj(1,9)], [vue4Proj(2,8) vue4Proj(2,9)], 'b');
plot([vue4Proj(1,10) vue4Proj(1,11)], [vue4Proj(2,10) vue4Proj(2,11)], 'b');
plot([vue4Proj(1,11) vue4Proj(1,12)], [vue4Proj(2,11) vue4Proj(2,12)], 'b');
plot([vue4Proj(1,1) vue4Proj(1,7)], [vue4Proj(2,1) vue4Proj(2,7)], 'b');
plot([vue4Proj(1,4) vue4Proj(1,10)], [vue4Proj(2,4) vue4Proj(2,10)], 'b');



%% Showing profiler to quantify efficiency of implementation
profile viewer  


%% Function for 3D to 2D 
function [p2D] = project3Dto2D(u,v,w,vue)
        init = [u;v;w; 1];                               
        prod = (vue.Kmat * vue.Pmat) * init;
        p2D = [prod(1)/prod(3); prod(2)/prod(3); 1];     
end
