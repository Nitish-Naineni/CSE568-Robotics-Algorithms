function [cors,cors_img] = harris(x)
    k = 0.05;
    I_x = gradient_x(x);
    I_y = gradient_y(x);
    
    Ixx = imgaussfilt(I_x.^2,1);
    Ixy = imgaussfilt(I_y.*I_x,1);
    Iyy = imgaussfilt(I_y.^2,1);
    detA = (Ixx.*Iyy) - Ixy.^2;
    traceA = Ixx + Iyy;
    R = detA - k * traceA.*2;
    maxima =  (R > imdilate(R, [1 1 1; 1 0 1; 1 1 1])).*R;
    %maxima = imregionalmax(R).*R;
    cors = zeros(200,2);
    cors_img = zeros(size(x));

    for i = 1:200
        [M,I] = max(maxima(:));
        [I_row, I_col] = ind2sub(size(maxima),I);
        cors(i,1) = I_row;
        cors(i,2) = I_col;
        cors_img(I_row,I_col) = 255;
        maxima(I_row,I_col) = 0;
    end
end

function I_x = gradient_x(frame)
    kernel_x = [-1,0,1;-2,0,2;-1,0,1];
    I_x = conv2(frame,kernel_x,'same');
end

function I_y = gradient_y(frame)
    kernel_y = [1,2,1;0,0,0;-1,-2,-1];
    I_y = conv2(frame,kernel_y,'same');
end