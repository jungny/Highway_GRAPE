% JSON 파일 경로
json_file = 'agentsetting.json';

[MEG, errors] = find_mutually_exclusive_groups(json_file);

% 결과 출력
disp('Mutually Exclusive Groups:');
disp(MEG);

% 순환 관계 에러 출력
if ~isempty(errors)
    disp('Errors:');
    disp(errors);
end