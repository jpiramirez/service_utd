% Batch load occupancy grids

G = load('poses.txt');
[rows,cols] = size(G);
A = zeros(rows, 40, 40);
for seq = 1:rows
  filein = sprintf('grid%04d.txt', seq-1);
  A(seq,:,:) = load(filein);  
end