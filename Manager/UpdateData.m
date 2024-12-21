function DataList = UpdateData(ObjectList,DataSize)
    
    DataList = zeros(size(ObjectList,1),DataSize);
    for i = 1:size(ObjectList,1)
        if ~isempty(ObjectList{i})
            DataList(i,1) = ObjectList{i}.ID;
            DataList(i,2) = ObjectList{i}.State;
            DataList(i,3) = ObjectList{i}.Lane;
            DataList(i,4) = ObjectList{i}.Location;
            DataList(i,5) = ObjectList{i}.Velocity;
            DataList(i,6) = ObjectList{i}.Exit;
        end
    end
    
end

