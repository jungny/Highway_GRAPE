function [MEG, errors] = find_mutually_exclusive_groups(json_file)
    % JSON 파일 읽기
    data = jsondecode(fileread(json_file));
    
    % 이웃 관계 저장
    agents = containers.Map;
    for i = 1:length(data)
        agent_id = data(i).ID;
        neighbours = data(i).neighbour_v_id;
        % 단일 값인 경우 배열로 변환
        if ~iscell(neighbours) && ~isvector(neighbours)
            neighbours = [neighbours];
        end
        % 숫자형 배열로 변환
        neighbours = double(neighbours);
        agents(num2str(agent_id)) = neighbours;
    end
    
    % 상호 이웃 관계 검사
    errors = {};  % 순환 관계에 대한 에러를 저장할 배열
    MEG = {};     % 결과로 반환될 mutually exclusive groups
    
    % 이미 확인한 노드를 추적
    visited = containers.Map;
    in_stack = containers.Map;  % 현재 DFS 스택에 있는 노드 추적
    
    % DFS 함수 정의
    function dfs(agent, group)
        % 현재 에이전트 방문 처리
        visited(agent) = true;
        in_stack(agent) = true;
        group = [group, str2double(agent)];
        
        % 모든 이웃 탐색
        neighbours = agents(agent);
        for i = 1:numel(neighbours)
            neighbour = num2str(neighbours(i));
            
            if ~isKey(visited, neighbour)
                dfs(neighbour, group);
            elseif isKey(in_stack, neighbour) && in_stack(neighbour)
                % 순환 관계 발견
                errors{end+1} = ['Circular relationship detected: ', agent, ' <-> ', neighbour];
            end
        end
        
        % DFS 스택에서 제거
        in_stack(agent) = false;
    end

    % 모든 에이전트에 대해 DFS 실행
    agent_keys = keys(agents);
    for i = 1:numel(agent_keys)
        agent = agent_keys{i};
        if ~isKey(visited, agent)
            group = [];
            dfs(agent, group);
            if ~isempty(group)
                MEG{end+1} = group;
            end
        end
    end
end
