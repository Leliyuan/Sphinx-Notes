% Copyright 2020 Delft University of Technology
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License. 

function exportToPreviousVersion(ModelName,Version)
%exportToPreviousVersion - Given simulink model is exported to a previous
% version. If the model contains referenced models (separate .slx files)
% these files are also tracked and exported to older versions. The Main
% simulink model is then adapted to refer to the files compatible in the
% same version.
%
% Syntax:  exportToPreviousVersion('ModelName','Version')
%
% Inputs:
%    ModelName  - Original model name 
%    Version    - Matlab version to export to
%
% Outputs:
%    Model              - Simulink model exported to given version
%    Referenced models  - Simulink model exported to given version
%
% Example: 
%    exportToPreviousVersion('Pointmass_eom_v4_1.slx','R2019B')
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Author: Dylan Eijkelhof, M.Sc.
% Delft University of Technology
% email address: d.eijkelhof@tudelft.nl  
% September 2020; Last revision: 04-September-2020

%------------- BEGIN CODE --------------

CopyFileName = [ModelName(1:end-4) '_tempcopy.slx'];
if exist(CopyFileName,'file')
    del_name = [pwd filesep CopyFileName];
    delete(del_name)
    pause(0.5)
    clearvars del_name
end
copyfile(ModelName,CopyFileName);

load_system(CopyFileName)
bl = Simulink.findBlocksOfType(CopyFileName(1:end-4),'ModelReference');
if ~isempty(bl)
for i = 1:numel(bl)
    paramValue = get_param(bl(i),'ModelFile');
    FullFile1 = which([paramValue(1:end-4) '_' Version '.slx']);
    if ~isempty(FullFile1)
        delete(FullFile1)
        pause(0.5)
    end
    FullFile = which(paramValue);
    [modelFilepath, modelName, ext] = fileparts(FullFile);
    load_system(FullFile)
    Simulink.exportToVersion(modelName,[modelName '_' Version ext],...
        Version,'BreakUserLinks',true);
    close_system(FullFile)
    movefile([modelName '_' Version ext],[modelFilepath filesep modelName '_' Version ext])
    set_param(bl(i),'ModelFile',[modelName '_' Version ext]);
end
end
if exist([ModelName(1:end-4) '_' Version '.slx'],'file')
    del_name = [pwd filesep ModelName(1:end-4) '_' Version '.slx'];
    delete(del_name)
    pause(0.5)
    clearvars del_name
end
save_system(CopyFileName,[],'OverwriteIfChangedOnDisk',true)
Simulink.exportToVersion(CopyFileName(1:end-4),[ModelName(1:end-4) '_' Version '.slx'],Version,'BreakUserLinks',true);
close_system(CopyFileName)

del_name = [pwd filesep CopyFileName];
delete(del_name)
pause(0.5)
end

%------------- END CODE --------------