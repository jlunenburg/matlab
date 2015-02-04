function goal = getSemanticGoal(env, goal_pose)

% Get x, y, phi of goal pose
xg = str2double(goal_pose.Attributes.x);
yg = str2double(goal_pose.Attributes.y);
phig = str2double(goal_pose.Attributes.phi);

%% Define goals
%if env == 'tue.library';
    goaltable = [
struct('goal','book_xxl','xlim',[-29.7800006866,-26.1599998474],'ylim',[7.83750009537,16.7124996185]),
struct('goal','cartographics','xlim',[19.7399997711,23.5599994659],'ylim',[-16.0524997711,-10.1000003815]),
struct('goal','copier','xlim',[3.70000004768,4.69999980927],'ylim',[7.97499990463,8.02499961853]),
struct('goal','elevator','xlim',[-11.0,-10.0],'ylim',[7.97499990463,8.02499961853]),
struct('goal','journals','xlim',[-29.8799991608,-26.0599994659],'ylim',[-16.2749996185,-7.41750001907]),
struct('goal','lecture_room_1','xlim',[-7.30000019073,-6.30000019073],'ylim',[7.97499990463,8.02499961853]),
struct('goal','lecture_room_2','xlim',[0.0,1.0],'ylim',[7.97499990463,8.02499961853]),
struct('goal','meeting_room_1','xlim',[9.27460193634,9.32539844513],'ylim',[-17.0000190735,-15.9999799728]),
struct('goal','meeting_room_4','xlim',[-15.5253982544,-15.4746017456],'ylim',[-17.0000190735,-15.9999799728]),
struct('goal','relax-room-1','xlim',[-29.8824996948,-26.1324996948],'ylim',[1.42449998856,5.02449989319]),
struct('goal','relax-room-2','xlim',[-29.8824996948,-26.1324996948],'ylim',[-4.43949985504,-0.839500010014]),
struct('goal','relax-room-3','xlim',[19.7374992371,23.4874992371],'ylim',[1.42449998856,5.02449989319]),
struct('goal','relax-room-4','xlim',[19.7374992371,23.4874992371],'ylim',[-4.43949985504,-0.839500010014]),
struct('goal','stairway','xlim',[-1.02539813519,-0.974601864815],'ylim',[-0.400019735098,0.600019752979]),
struct('goal','theses','xlim',[19.8400001526,23.4599990845],'ylim',[8.04749965668,16.8525009155]),
];
%end

goal = 'none';
for ii = 1:length(goaltable);
    xlim = goaltable(ii).xlim; xlim(1) = xlim(1) - 1.1; xlim(2) = xlim(2) + 1.1;
    ylim = goaltable(ii).ylim; ylim(1) = ylim(1) - 1.1; ylim(2) = ylim(2) + 1.1;
    if (xlim(1) <= xg && xg <= xlim(2) && ylim(1) <= yg && yg <= ylim(2))
        goal = goaltable(ii).goal;
        break
    end
end

% if strcmp(goal, 'none')
%     fprintf('No goal found for [x = %f, y = %f]\n', xg, yg)
% else
%     fprintf('Endpose [x = %f, y = %f] between x = [%f, %f], y = [%f, %f]\n', xg, xlim(1), xlim(2), yg, ylim(1), ylim(2))
% end

%goal;"-2.082" x="-24.13" y="16.732" />

%% Made using:
% for entity in entities:
%     cv = entity.convex_hull
%     if len(cv) > 0:
%         xmin = cv[0].x
%         xmax = cv[0].x
%         ymin = cv[0].y
%         ymax = cv[0].y
%         for c in cv:
%             xmin = min(xmin,c.x)
%             xmax = max(xmax,c.x)
%             ymin = min(ymin,c.y)
%             ymax = max(ymax,c.y)
%         print "struct('goal','{0}','xlim',[{1},{2}],'ylim',[{3},{4}]),".format(entity.id,xmin,xmax,ymin,ymax)
        