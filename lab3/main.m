close all
clc

for i = 1:6
    input_filename = sprintf("data/image%d.jpg",i);
    img = double(imread(input_filename));
    b = img(25:325,25:370);
    g = img(365:665,25:370);
    r = img(695:995,25:370);
    img_color = cat(3,uint8(r),uint8(g),uint8(b));
    output_filename = sprintf("color/image%d-color.jpg",i);
    imwrite(img_color,output_filename)
    im_align1(r,g,b,i);
    im_align2(r,g,b,i);
    im_align3(r,g,b,i);
    
end
