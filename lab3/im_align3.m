function [] = im_align3(r,g,b,i)
    
    
    [cors_r,cors_img_r] = harris(r);
    [cors_g,cors_img_g] = harris(g);
    [cors_b,cors_img_b] = harris(b);

    offset_g = ransac(cors_b,cors_g,5000);
    offset_r = ransac(cors_b,cors_r,5000);
    shifted_r = circshift(r,[offset_r(1),offset_r(2)]);
    shifted_g = circshift(g,[offset_g(1),offset_g(2)]);

    img_corner = cat(3,uint8(shifted_r), uint8(shifted_g), uint8(b));
    
    output_filename_corner = sprintf("corner/image%d-corner.jpg",i);
    imwrite(img_corner,output_filename_corner)
    fprintf("aligned image%d using corner, shifted r by (%d,%d) and g by (%d,%d)\n",i,offset_r(1),offset_r(2),offset_g(1),offset_g(2))
end

function best_offset = ransac(cors_b,cors_g,iters)
    best_ins = 0;
    best_offset = [0,0];
    for i = 1:iters
        sel_b = cors_b(ceil(rand * 200),:);
        near_g = {};
        for j = 1:200
            if (abs(cors_g(j,1)-sel_b(1)) < 10) && (abs(cors_g(j,2)-sel_b(2)) < 10)
                near_g = [near_g,cors_g(j,:)];
            end
        end
        sz = size(near_g);
        if sz(2) == 0
            continue
        end
        sel_g = near_g{ceil(rand * sz(2))};
        offset = sel_b - sel_g;
        shifted_cors_g = cors_g + offset;
        ins = 0;
        for j = 1:200
            if 2 >= min(prod(abs(shifted_cors_g - cors_b(j)),2))
                ins = ins + 1;
            end
        end
        
        if best_ins < ins
            best_ins = ins;
            best_offset = offset;
        end

    end
end