function sc = covarianceEllipse1D(c,P,color,scale)
% covarianceEllipse1D plots a Gaussian as an uncertainty bound in 1D plots
% Based on Maybeck Vol 1, page 366
% k=2.296 corresponds to 1 std, 68.26% of all probability
% k=11.82 corresponds to 3 std, 99.74% of all probability
%
% Modified from http://www.mathworks.com/matlabcentral/newsreader/view_thread/42966

hold on

[e,s] = svd(P);
k = 2.296; 
radii = k*sqrt(diag(s));

if exist('scale', 'var')
    radii = radii * scale;
end

errorbar(c(1), c(2), nthroot(radii(1)*radii(2)*radii(3), 3), 'Color', color);

end

