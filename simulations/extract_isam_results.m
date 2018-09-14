res_file = importdata('isam/_Aux/temp_isam_incremental.txt');
% res_file = importdata('isam/_Aux/temp_isam');
NbL = length(res_file);
res_poses = zeros(3,NbL);
iPoses = 0;
for i=1:NbL  
    line = res_file{i};
    ind=find(line==' ');
    header = line(1:(ind(1)-1));
    if strcmp(header,'Pose2d_Node')
        iPoses = iPoses+1;
        posx = str2double(line((ind(2)+2):ind(3)-2));
        posy = str2double(line((ind(3)+1):ind(4)-2));
        postheta = str2double(line((ind(4)+1):end-1));
        res_poses(:,iPoses)=[postheta;posx;posy];
    end   
end
res_poses = res_poses(:,2:iPoses);

% extract isam cov
res_file = importdata('isam/_Aux/temp_isam_cov.txt');
NbL = length(res_file);
Pe_isam = zeros(3,3,nSteps);
for i = 4:3:NbL
   iStep = (i-1)/3;
   P_aux = res_file(i:i+2,:);
   Pe_isam(:,:, iStep) = P_aux(1:3, 1:3);
end


