endp=load('end1.txt');
endp=reshape(endp,[3 size(endp,1)/3]);
scatter3(endp(1,:),endp(2,:),endp(3,:),20,'filled');
axis equal
hold on
endp2=load('end2.txt');
endp2=reshape(endp2,[3 size(endp2,1)/3]);
scatter3(endp2(1,:),endp2(2,:),endp2(3,:),20,'filled');
axis equal

xlabel('x');
ylabel('y');
zlabel('z');

legend('Before BA','After BA')

disp(['X-axis variance before BA: ' num2str(var(endp(1,:)))])
disp(['Y-axis variance before BA: ' num2str(var(endp(2,:)))])
disp(['Z-axis variance before BA: ' num2str(var(endp(3,:)))])
disp(['X-axis variance after BA: ' num2str(var(endp2(1,:)))])
disp(['Y-axis variance after BA: ' num2str(var(endp2(2,:)))])
disp(['Z-axis variance after BA: ' num2str(var(endp2(3,:)))])

R=load('rot2.txt');
t=load('trans2.txt');
R=[R repmat(-eye(3),[size(R,1)/3 1])];
x=R\(-t);