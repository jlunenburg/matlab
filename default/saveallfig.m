function saveallfig(name,handle,path) 
% saveallfig(name, handle, path)
% 
%path settings: You can give your favorite paths here, I always set it to the pictures folder of my current project
%               just enter a number in the 'path' input instead of the path
preferredpath{1}='D:\Thesis\'; 
%preferredpath{2}='C:\Pictures\;
%etc,etc...'

%use current figure if no handle is supplied 
if (nargin<2) || isempty(handle)
    handle=gcf; 
end 
%default path current dir 
if (nargin<3) || isempty(path) 
    if(isunix)         
        path=[cd '/']; %for linux users 
    else 
        path=[cd '\']; %for windows users 
    end
else
    if isnumeric(path) %if path is one of the preferred paths
        path=preferredpath{path};     
    else
        %do nothing, path given in function input is the desired path
    end       
end

% Save figure
saveas(handle, strcat(path, strcat('/', strcat(name,'.fig') ) ) );

% Save pdf
saver(name, handle, path);

% Save tikz
matlab2tikz('figurehandle',handle,'filename', strcat(path, strcat('/', strcat(name,'.tikz') ) ),...
    'height','\figureheight','width','\figurewidth',...
    'parseStrings',false);
fprintf('\n*** Please check the x- and y-labels of the output\n')

