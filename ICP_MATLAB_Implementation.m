close all
clear all
clc
ptCloudRef = pcread('bun045 (1).ply');
ptCloudCurrent = pcread('bun090 (1).ply');
% pcshowpair(ptCloudRef,ptCloudCurrent);
% clear data
ptCloudRef = pcdenoise(ptCloudRef);
ptCloudCurrent = pcdenoise(ptCloudCurrent);
mod = transpose(ptCloudCurrent.Location);
dat = transpose(ptCloudRef.Location); 
model = double(mod);
data = double(dat);

figure(1)
plot3(model(1,:),model(2,:),model(3,:),'r.',data(1,:),data(2,:),data(3,:),'b.'), hold on, axis equal
plot3([0.05 0.05 0],[0 0.05 0.05],[0 0 0],'r-',[0.05 0.05],[0.05 0.05],[0 0.05],'r-','LineWidth',2)
title('Original data points (blue) and model points (red)')

ptCloud1 = pointCloud(ptCloudCurrent.Location);
ptCloud2 = pointCloud(ptCloudRef.Location);


% Running the ICP-algorithm. Least squares criterion

[RotMat,TransVec,dataOut]=ICP_NEW(model,data);

% Reference:
%
% Bergström, P. and Edlund, O. 2014, 'Robust registration of point sets using iteratively reweighted least squares'
% Computational Optimization and Applications, vol 58, no. 3, pp. 543-561, 10.1007/s10589-014-9643-2


% A plot. Model points and data points in transformed positions

figure(2)
plot3(model(1,:),model(2,:),model(3,:),'r.',dataOut(1,:),dataOut(2,:),dataOut(3,:),'g.'), hold on, axis equal
plot3([0.05 0.05 0],[0 0.05 0.05],[0 0 0],'r-',[0.05 0.05],[0.05 0.05],[0 0.05],'r-','LineWidth',2)
title('Transformed data points (green) and model points (red)')
ptCloud3 = pointCloud(transpose(dataOut));
% figure(3)
% pcshow(ptCloud1, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% hold on
% pcshow(ptCloud2, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% title('First input')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% drawnow
% figure(4)
% pcshow(ptCloud1, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% hold on
% pcshow(ptCloud3, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% title('First input')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% drawnow
figure(5)
pcshowpair(ptCloud1,ptCloud2,'VerticalAxis','Y','VerticalAxisDir','Down')
title('Initial Point Clouds')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')

figure(6)
pcshowpair(ptCloud1,ptCloud3,'VerticalAxis','Y','VerticalAxisDir','Down')
title('Point Clouds after ICP')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
% player = pcplayer([0 1],[0 1],[0 1]); 
% ptCloud = po` QintCloud(model);0.05
% view(player,ptCloud); 
% figure
% pcshowpair(model,dataOut,'VerticalAxis','Y','Vertical