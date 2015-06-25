# Visualization script for Octave

outseq = 1;
for images = 1:1000
  filein = sprintf('grid%04d.txt', images);
  fileout = sprintf('out%04d.png', outseq);
  A = load(filein);
  imagesc(log(A+1e-8)); colormap(bone);
  print(fileout);
  outseq = outseq + 1;
end