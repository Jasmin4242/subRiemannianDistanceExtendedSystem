%
% Save given data struct as table for pgfplots.
% A. Kargl, 05/2023
%
function data2pgftable(path, data, varargin)

%% Optinal Input
% default values
skip = 1;

% go through optionals
if nargin > 2
    for i = 1:2:nargin-2

        % argument name and value
        argname = varargin{i};
        argval = varargin{i+1};

        % apply values
        switch argname
            case 'skip'
                skip = argval;
                break
        end
    end
end

%% Open file
% add ending if not present
[~, ~, fEnding] = fileparts(path);

if isempty(fEnding)
    path = [path, '.txt'];
end

% open file (in overwrite mode)
fHandle = fopen(path, "w+");

%% Save data
% save every field of struct as column
colNames = fieldnames(data);
M = length(colNames);
N = zeros(1, M);
for j = 1:M
    N(j) = length(data.(colNames{j}));
end

% fieldnames in header row
for j = 1:M
    fprintf(fHandle, ['%s', blanks(4)], colNames{j});
end
fprintf(fHandle, [newline, '#', newline]);

% data
for i = 1:skip:max(N)
    for j = 1:M
        if i <= N(j)
            fprintf(fHandle, ['%+.6e', blanks(4)], data.(colNames{j})(i));
        else
            fprintf(fHandle, [blanks(10), 'nan', blanks(4)]);
        end
    end
    fprintf(fHandle, newline);
end

%% Close file
fclose(fHandle);

end
