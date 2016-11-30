fid = fopen('DB_AVG.txt','w');
DB_Array = table2array(avgDB);
[r,c] = size(DB_Array);
for i = 1:r
    for j = 1:c
        if j == c
            fprintf(fid,'%0.2f',DB_Array(i,j));
        else
            fprintf(fid,'%0.2f,',DB_Array(i,j));
        end
    end
    fprintf(fid,'\n');
end
fclose(fid);