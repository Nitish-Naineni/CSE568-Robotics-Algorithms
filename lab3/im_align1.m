function [] = im_align1(r,g,b,i)
    sz = size(r);
    crop = 15;
    norm_b = norm(b);
    norm_r = norm(r);
    norm_g = norm(g);
    crop_b = b(crop:sz(1)-crop,crop:sz(2)-crop)/norm_b;

    best_case_r = [1,1,realmax];
    best_case_g = [1,1,realmax];

    for m = -crop:crop
        for n = -crop:crop
            shifted_r = circshift(r,[m,n]);
            shifted_g = circshift(g,[m,n]);

            crop_r = shifted_r(crop:sz(1)-crop,crop:sz(2)-crop)/norm_r;
            crop_g = shifted_g(crop:sz(1)-crop,crop:sz(2)-crop)/norm_g;

            simi_r = sum((crop_b-crop_r).^2,"all");
            simi_g = sum((crop_b-crop_g).^2,"all");

            if simi_r < best_case_r(3)
                best_case_r = [m,n,simi_r];
            end
            if simi_g < best_case_g(3)
                best_case_g = [m,n,simi_g];
            end
        end
    end
    shifted_r = circshift(r,[best_case_r(1),best_case_r(2)]);
    shifted_g = circshift(g,[best_case_g(1),best_case_g(2)]);

    img_ssd = cat(3,uint8(shifted_r), uint8(shifted_g), uint8(b));
    
    output_filename_ssd = sprintf("ssd/image%d-ssd.jpg",i);
    imwrite(img_ssd,output_filename_ssd)
    fprintf("aligned image%d using SSD,    shifted r by (%d,%d) and g by (%d,%d)\n",i,best_case_r(1),best_case_r(2),best_case_g(1),best_case_g(2))
end




