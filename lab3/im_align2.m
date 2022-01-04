function [] = im_align2(r,g,b,i)
    sz = size(r);
    crop = 15;
    crop_b = b(crop:sz(1)-crop,crop:sz(2)-crop);

    best_case_r = [1,1,0];
    best_case_g = [1,1,0];

    for m = -crop:crop
        for n = -crop:crop
            shifted_r = circshift(r,[m,n]);
            shifted_g = circshift(g,[m,n]);

            crop_r = shifted_r(crop:sz(1)-crop,crop:sz(2)-crop);
            crop_g = shifted_g(crop:sz(1)-crop,crop:sz(2)-crop);

            simi_r = sum(crop_b .* crop_r,"all") / sqrt(sum(crop_b.^2,"all")*sum(crop_r.^2,"all"));
            simi_g = sum(crop_b .* crop_g,"all") / sqrt(sum(crop_b.^2,"all")*sum(crop_g.^2,"all"));

            if simi_r > best_case_r(3)
                best_case_r = [m,n,simi_r];
            end
            if simi_g > best_case_g(3)
                best_case_g = [m,n,simi_g];
            end
        end
    end
    shifted_r = circshift(r,[best_case_r(1),best_case_r(2)]);
    shifted_g = circshift(g,[best_case_g(1),best_case_g(2)]);

    img_ncc = cat(3,uint8(shifted_r), uint8(shifted_g), uint8(b));
    
    output_filename_ncc = sprintf("ncc/image%d-ncc.jpg",i);
    imwrite(img_ncc,output_filename_ncc)
    fprintf("aligned image%d using NCC,    shifted r by (%d,%d) and g by (%d,%d)\n",i,best_case_r(1),best_case_r(2),best_case_g(1),best_case_g(2))
end




