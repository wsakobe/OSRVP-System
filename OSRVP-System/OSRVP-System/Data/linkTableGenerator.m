% T = [0,0,0,1,0,1,0,1,0,1,1,1,0,0,1,0,1,0;0,0,0,1,1,1,0,1,0,0,1,1,0,0,0,1,0,0;0,0,1,0,1,0,0,0,1,1,1,0,1,1,0,0,1,1;0,1,1,0,0,0,1,1,1,1,1,0,1,1,1,1,0,1;1,0,1,0,0,0,0,1,0,0,1,0,0,1,0,1,0,1;0,1,1,1,1,1,1,0,0,0,0,1,0,1,0,0,0,1];
T=sta;
global cnt;
global linkTable;
linkTable = [];
cnt = 1;

for rot = 1:4
    for i = 1 : size(T , 1)
        for j = 1 : size(T , 2)
            if (i<size(T,1)-1) && (j<size(T,2)-1)
                extractMatrix(i, j, i+2, j+2, T, 1, rot);
            end
%             if (i == 1)
%                 extractMatrix(i, j, i+9, j, T, 2, rot);
%             end
%             if (i<size(T,1)) && (j<size(T,2)-4)
%                 extractMatrix(i, j, i+1, j+5, T, 3, rot);
%             end
        end
    end
    T=rot90(T,1);
end

linkTable = [linkTable.score; linkTable.x; linkTable.y; linkTable.rotate]';

function extractMatrix(i1, j1, i2, j2, T, mode, rot)
    global cnt;
    global linkTable;
    tempMatrix = T(i1:i2,j1:j2);
    score=0;
    binary = 1;
    for ia = 1:size(tempMatrix,1)
        for ib = 1:size(tempMatrix,2)
            if (tempMatrix(ia,ib)==-1)
                score = NaN;
                break;
            end
            if (tempMatrix(ia,ib)==1)
                score = score + binary;
            end
            binary = binary * 2;
        end
    end

    switch (rot)
        case 1
            linkTable(cnt).y=i1-1;
            linkTable(cnt).x=j1-1;
        case 2
            linkTable(cnt).y=j1-1;
            linkTable(cnt).x=size(T,1) - i1;
        case 3
            linkTable(cnt).y=size(T,1) - i1;
            linkTable(cnt).x=size(T,2) - j1;
        case 4
            linkTable(cnt).y=size(T,2) - j1;
            linkTable(cnt).x=i1-1;
    end
    if mod(linkTable(cnt).x + linkTable(cnt).y, 2) == 0 %1代表左上角棋盘格底色为白色，0代表为黑色
        score = score + binary;
    end
    switch (rot)
        case 2
            linkTable(cnt).x=linkTable(cnt).x + 1;
        case 3
            linkTable(cnt).x=linkTable(cnt).x + 1;
            linkTable(cnt).y=linkTable(cnt).y + 1;
        case 4
            linkTable(cnt).y=linkTable(cnt).y + 1;
    end

    linkTable(cnt).score=score;
    linkTable(cnt).mode=mode;
    linkTable(cnt).rotate=rot-1;
    cnt = cnt + 1;
end